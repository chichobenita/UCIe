// axi_bridge_ip_tx_tb_pkg.sv
package axi_bridge_ip_tx_tb_pkg;

  // אותם פרמטרים כמו ב-DUT (לעכשיו)
  parameter int DATA_W   = 256;
  parameter int IF_W     = 64;
  parameter int TUSER_W  = 16;

  localparam int BYTES_PER_BEAT = DATA_W / 8;
  localparam int BYTES_PER_SEG  = IF_W   / 8;

  // -----------------------------
  // טיפוסים ל-beat ול-frame בצד ה-IP
  // -----------------------------
  typedef struct {
    logic [DATA_W-1:0]         data;
    logic [BYTES_PER_BEAT-1:0] keep;
    logic                      last;
    logic [TUSER_W-1:0]        user;
  } axi_beat_t;

  class axi_frame;
    // כל frame מורכב מרשימת beats
    axi_beat_t beats[$];

    function void add_beat(axi_beat_t b);
      beats.push_back(b);
    endfunction
  endclass

  // -----------------------------
  // טיפוס לסגמנט בצד ה-Client
  // -----------------------------
  typedef struct {
    logic [IF_W-1:0]           data;
    logic [BYTES_PER_SEG-1:0]  keep;
    logic [TUSER_W-1:0]        user;
    logic                      sop;
    logic                      eop;
  } tx_seg_t;

  // -----------------------------
  // Reference Model:
  // מקבל frame ומחזיר רשימת segments צפויים
  // -----------------------------
function automatic void ref_build_segments(
    input  axi_frame   frame,
    output tx_seg_t    segs[$]
);
  // --- הכרזות בראש הפונקציה בלבד ---
  byte unsigned        bytes[$];      // אוסף את כל הבייטים התקפים ברצף
  logic [TUSER_W-1:0]  frame_user;
  int                  num_bytes;
  int                  num_segs;
  int                  i, j, s, bb, idx;
  axi_beat_t           beat;
  byte unsigned        tmp;
  tx_seg_t             seg;

  // נתחיל עבודה
  segs.delete();

  if (frame.beats.size() == 0) return;

  frame_user = frame.beats[0].user;

  // 1. אוספים בייטים מתוך ה-beats לפי keep
  foreach (frame.beats[i]) begin
    beat = frame.beats[i];
    for (j = 0; j < BYTES_PER_BEAT; j++) begin
      if (beat.keep[j]) begin
        // byte j = bits [8*j +: 8] – אותו כיווניות כמו ב-DUT
        tmp = beat.data[j*8 +: 8];
        bytes.push_back(tmp);
      end
    end
  end

  num_bytes = bytes.size();
  if (num_bytes == 0) return;

  num_segs  = (num_bytes + BYTES_PER_SEG - 1) / BYTES_PER_SEG;

  // 2. מחלקים לבלאקים של BYTES_PER_SEG ומייצרים segments
  for (s = 0; s < num_segs; s++) begin
    // מאפסים seg
    seg.data = '0;
    seg.keep = '0;
    seg.user = frame_user;
    seg.sop  = (s == 0);
    seg.eop  = (s == num_segs-1);

    for (bb = 0; bb < BYTES_PER_SEG; bb++) begin
      idx = s*BYTES_PER_SEG + bb;
      if (idx < num_bytes) begin
        seg.data[bb*8 +: 8] = bytes[idx];
        seg.keep[bb]        = 1'b1;
      end
    end

    segs.push_back(seg);
  end
endfunction

  // --------------------------------------------------
  // Driver – מוציא frame על אינטרפייס ה-AXI-Stream
  // --------------------------------------------------
  // ההגדרה של axi_stream_if נמצאת בקובץ אחר, אבל אנחנו
  // משתמשים בשמה כאן (modport drv).
typedef virtual axi_stream_if #(DATA_W, TUSER_W) drv_axi_vif_t;

class axi_driver;
  drv_axi_vif_t vif;

  function new(drv_axi_vif_t vif);
    this.vif = vif;
  endfunction

  task drive_frame(axi_frame f);
    // דיפולט
    vif.tvalid <= 1'b0;
    vif.tlast  <= 1'b0;
    vif.tdata  <= '0;
    vif.tkeep  <= '0;
    vif.tuser  <= '0;

    foreach (f.beats[i]) begin
      axi_beat_t b = f.beats[i];

      // מחכים לפוז' כדי ליישר לשעון
      @(posedge vif.clk);

      // מעמיסים beat על הקווים ומעלים valid
      vif.tdata  <= b.data;
      vif.tkeep  <= b.keep;
      vif.tuser  <= b.user;
      vif.tlast  <= b.last;
      vif.tvalid <= 1'b1;

      // עכשיו מחכים ל-handshake אמיתי:
      // valid=1 כבר, מחכים עד ש-tready יהיה 1 באחד הפוז'ים.
      do begin
        @(posedge vif.clk);
      end while (!vif.tready);

      // בפוז' האחרון קרה ה-handshake (valid=1 && ready=1),
      // עכשיו אנחנו מורידים valid כדי שלא יהיה עוד handshake על אותו beat.
      vif.tvalid <= 1'b0;
      vif.tlast  <= 1'b0;
    end
  endtask
endclass

  // --------------------------------------------------
  // Monitor – אוסף segments בצד ה-Client-IF
  // --------------------------------------------------
typedef virtual client_if #(IF_W, TUSER_W) mon_client_vif_t;

class client_monitor;
  mon_client_vif_t vif;
  tx_seg_t         seg_q[$];

  function new(mon_client_vif_t vif);
    this.vif = vif;
  endfunction

  task run();
    forever begin
      @(vif.cb);  // אירוע clocking – אחרי #1step
      if (vif.cb.valid && vif.cb.ready) begin
        tx_seg_t s;
        s.data = vif.cb.data;
        s.keep = vif.cb.keep;
        s.user = vif.cb.user;
        s.sop  = vif.cb.sop;
        s.eop  = vif.cb.eop;
        seg_q.push_back(s);
        $display("[%0t] MON: data=%h sop=%0b eop=%0b",
                 $time, s.data, s.sop, s.eop);
      end
    end
  endtask
endclass



  // --------------------------------------------------
  // Scoreboard – משווה expected מול actual
  // --------------------------------------------------
class scoreboard;
  client_monitor mon;

  function new(client_monitor mon_i);
    this.mon = mon_i;
  endfunction

  task automatic check_frame(axi_frame f);
    tx_seg_t exp_segs[$];
    tx_seg_t exp, act;
    int      i;

    ref_build_segments(f, exp_segs);

    // מחכים עד שנקבל לפחות את מספר הסגמנטים הצפוי
    wait (mon.seg_q.size() >= exp_segs.size());

    // השוואה אחד-לאחד
    foreach (exp_segs[i]) begin
    tx_seg_t exp, act;
    exp = exp_segs[i];
    act = mon.seg_q.pop_front();  // לוקח את הסגמנט הבא מתוך התור

    if (act.data !== exp.data   ||
        act.keep !== exp.keep   ||
        act.user !== exp.user   ||
        act.sop  !== exp.sop    ||
        act.eop  !== exp.eop) begin
      $error("Scoreboard mismatch at segment %0d", i);
        $display("  EXP: data=%h keep=%b user=%h sop=%0b eop=%0b",
                  exp.data, exp.keep, exp.user, exp.sop, exp.eop);
        $display("  ACT: data=%h keep=%b user=%h sop=%0b eop=%0b",
                  act.data, act.keep, act.user, act.sop, act.eop);
      end
    end

    $display("[SCOREBOARD] Frame OK, %0d segments", exp_segs.size());
  endtask
  
    function void check_no_extra_segments(string tag = "");
    if (mon.seg_q.size() != 0) begin
      $error("Scoreboard: extra segments left after %s, seg_q.size() = %0d",
             tag, mon.seg_q.size());
    end
  endfunction

endclass

endpackage
