import { useState } from "react";

const C = {
  bg: "#080c14", panel: "#0d1424", panelBorder: "#1a2540",
  accent: "#00d4ff", accentDim: "#0891b2",
  orange: "#f59e0b", orangeDim: "#d97706",
  green: "#10b981", greenDim: "#059669",
  red: "#ef4444", redDim: "#dc2626",
  purple: "#a78bfa", purpleDim: "#7c3aed",
  yellow: "#fbbf24",
  textDim: "#94a3b8", textMuted: "#4a5568",
  new: "#34d399", newDim: "#059669",
};

const tabs = [
  { id: "thread",    label: "Thread Model" },
  { id: "dataflow",  label: "Data Flow ★" },
  { id: "component", label: "Component" },
  { id: "sequence",  label: "Sequence" },
  { id: "state",     label: "State Machine" },
];

function Defs() {
  return (
    <defs>
      <filter id="glow">
        <feGaussianBlur stdDeviation="2.5" result="blur"/>
        <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
      </filter>
      {[["aC",C.accent],["aO",C.orange],["aG",C.green],["aP",C.purple],
        ["aR",C.red],["aDim",C.textDim],["aN",C.new],["aY",C.yellow]].map(([id,col])=>(
        <marker key={id} id={id} viewBox="0 0 10 7" refX="9" refY="3.5"
          markerWidth="7" markerHeight="5" orient="auto-start-reverse">
          <polygon points="0 0,10 3.5,0 7" fill={col}/>
        </marker>
      ))}
      <linearGradient id="prioGrad" x1="0%" y1="0%" x2="0%" y2="100%">
        <stop offset="0%"   stopColor={C.green}  stopOpacity="0.2"/>
        <stop offset="50%"  stopColor={C.orange} stopOpacity="0.06"/>
        <stop offset="100%" stopColor={C.red}    stopOpacity="0.2"/>
      </linearGradient>
    </defs>
  );
}

// ─── helper: labelled arrow segment ─────────────────────────────────────────
function Arrow({d, col, marker, label, lx, ly, dashed=false}) {
  return (
    <g>
      <path d={d} fill="none" stroke={col} strokeWidth="1.4"
        strokeDasharray={dashed ? "5 3" : undefined}
        markerEnd={`url(#${marker})`}/>
      {label && <text x={lx} y={ly} fill={col} fontSize="9"
        fontFamily="monospace" textAnchor="middle">{label}</text>}
    </g>
  );
}

// ─── THREAD MODEL ────────────────────────────────────────────────────────────
// Layout:  viewBox 1100 x 760
//  HIGH band  y 65-130   → (empty — no HIGH-prio thread in this app)
//  MED  band  y 130-430  → main, can_tx_100ms, can_tx_1000ms
//  LOW  band  y 430-700  → lvgl_thread (tall box, fully visible)
//  legend     y 720-748
function ThreadDiagram() {
  return (
    <svg viewBox="0 0 1100 760" style={{width:"100%",height:"auto"}}>
      <Defs/>
      <text x="550" y="28" textAnchor="middle" fill={C.accent}
        fontSize="15" fontFamily="monospace" fontWeight="bold" filter="url(#glow)">
        Thread Priority &amp; Scheduling Model — v3 (pós-diff)
      </text>
      <text x="550" y="46" textAnchor="middle" fill={C.textMuted}
        fontSize="10" fontFamily="monospace">NuttX Preemptive Scheduler · 4 pthreads</text>

      {/* priority gradient bg */}
      <rect x="80" y="60" width="960" height="645" rx="4"
        fill="url(#prioGrad)" stroke={C.panelBorder} strokeWidth="1"/>

      {/* band lines + labels */}
      <line x1="80" y1="90"  x2="1040" y2="90"  stroke={C.green}  strokeWidth="0.5" strokeDasharray="4 3"/>
      <line x1="80" y1="300" x2="1040" y2="300" stroke={C.orange} strokeWidth="0.5" strokeDasharray="4 3"/>
      <line x1="80" y1="490" x2="1040" y2="490" stroke={C.red}    strokeWidth="0.5" strokeDasharray="4 3"/>
      <text x="62" y="95"  fill={C.green}  fontSize="9" fontFamily="monospace" textAnchor="end">HIGH</text>
      <text x="62" y="305" fill={C.orange} fontSize="9" fontFamily="monospace" textAnchor="end">MED</text>
      <text x="62" y="495" fill={C.red}    fontSize="9" fontFamily="monospace" textAnchor="end">LOW</text>

      {/* ── KEY INSIGHT (HIGH band) ── */}
      <rect x="100" y="68" width="840" height="190" rx="6"
        fill={C.panel} stroke={C.accentDim} strokeWidth="1"/>
      <text x="520" y="90" textAnchor="middle" fill={C.accent}
        fontSize="11" fontFamily="monospace" fontWeight="bold">MUDANÇA ARQUITETURAL — pós-diff</text>
      <line x1="115" y1="96" x2="925" y2="96" stroke={C.accentDim} strokeWidth="0.5"/>
      {/* two columns */}
      <text x="200" y="116" textAnchor="middle" fill={C.red} fontSize="10" fontFamily="monospace" fontWeight="bold">ANTES</text>
      <text x="200" y="134" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">main loop: select + CAN + btns + LVGL</text>
      <text x="200" y="151" textAnchor="middle" fill={C.red} fontSize="9" fontFamily="monospace">→ CAN flood = LVGL travado</text>
      <text x="200" y="168" textAnchor="middle" fill={C.red} fontSize="9" fontFamily="monospace">→ widgets_set_*() de threads = crash</text>
      <line x1="430" y1="104" x2="430" y2="248" stroke={C.panelBorder} strokeWidth="1"/>
      <text x="680" y="116" textAnchor="middle" fill={C.new} fontSize="10" fontFamily="monospace" fontWeight="bold">DEPOIS</text>
      <text x="680" y="134" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">main escreve globals → g_ui_dirty</text>
      <text x="680" y="151" textAnchor="middle" fill={C.new} fontSize="9" fontFamily="monospace">→ zero race conditions no LVGL</text>
      <text x="680" y="168" textAnchor="middle" fill={C.new} fontSize="9" fontFamily="monospace">→ lvgl_thread: único dono do LVGL</text>
      <text x="680" y="185" textAnchor="middle" fill={C.new} fontSize="9" fontFamily="monospace">→ LVGL sempre responsivo</text>

      {/* ── MED BAND: main ── */}
      <rect x="100" y="310" width="240" height="160" rx="6"
        fill={C.panel} stroke={C.accent} strokeWidth="1.5"/>
      <text x="220" y="334" textAnchor="middle" fill={C.accent}
        fontSize="12" fontFamily="monospace" fontWeight="bold">main() loop</text>
      <text x="220" y="352" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">Priority: DEFAULT</text>
      <text x="220" y="368" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">select(cmd_fd, 10ms timeout)</text>
      <text x="220" y="384" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">poll buttons</text>
      <text x="220" y="400" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">process_can_frame()</text>
      <text x="220" y="416" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">process_button_press()</text>
      <rect x="116" y="426" width="208" height="22" rx="4"
        fill={C.new+"22"} stroke={C.new} strokeWidth="1"/>
      <text x="220" y="441" textAnchor="middle" fill={C.new}
        fontSize="9" fontFamily="monospace" fontWeight="bold">✗ NÃO chama LVGL diretamente</text>

      {/* ── MED BAND: can_tx_100ms ── */}
      <rect x="390" y="310" width="200" height="160" rx="6"
        fill={C.panel} stroke={C.orange} strokeWidth="1.5"/>
      <text x="490" y="334" textAnchor="middle" fill={C.orange}
        fontSize="11" fontFamily="monospace" fontWeight="bold">can_tx_100ms</text>
      <text x="490" y="352" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">Priority: DEFAULT</text>
      <text x="490" y="368" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">Stack: 2048 B</text>
      <text x="490" y="384" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">DRIVER_COMMANDS</text>
      <text x="490" y="400" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">nxsched_msleep(100)</text>

      {/* ── MED BAND: can_tx_1000ms ── */}
      <rect x="640" y="310" width="200" height="160" rx="6"
        fill={C.panel} stroke={C.orange} strokeWidth="1.5"/>
      <text x="740" y="334" textAnchor="middle" fill={C.orange}
        fontSize="11" fontFamily="monospace" fontWeight="bold">can_tx_1000ms</text>
      <text x="740" y="352" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">Priority: DEFAULT</text>
      <text x="740" y="368" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">Stack: 2048 B</text>
      <text x="740" y="384" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">HMI_INFO heartbeat</text>
      <text x="740" y="400" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">nxsched_msleep(1000)</text>

      {/* ── LOW BAND: lvgl_thread ── fully inside viewBox ── */}
      <rect x="390" y="510" width="560" height="190" rx="6"
        fill={C.panel} stroke={C.new} strokeWidth="2"/>
      {/* "NOVO no diff" badge */}
      <rect x="390" y="498" width="120" height="20" rx="4" fill={C.new}/>
      <text x="450" y="512" textAnchor="middle" fill={C.bg}
        fontSize="9" fontFamily="monospace" fontWeight="bold">NOVO no diff</text>
      <text x="670" y="540" textAnchor="middle" fill={C.new}
        fontSize="14" fontFamily="monospace" fontWeight="bold">lvgl_thread</text>
      <text x="670" y="558" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">Priority: DEFAULT - 10  ·  Stack: 4096 B</text>
      {/* two columns inside */}
      <text x="510" y="580" textAnchor="middle" fill={C.new} fontSize="9" fontFamily="monospace" fontWeight="bold">O que faz:</text>
      <text x="510" y="597" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">1. if (g_ui_dirty) → snapshot</text>
      <text x="510" y="613" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">     → widgets_set_*(snap)</text>
      <text x="510" y="629" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">2. if (g_screen_request) → load()</text>
      <text x="510" y="645" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">3. lv_tick_inc(10)</text>
      <text x="510" y="661" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">4. lv_timer_handler()</text>
      <line x1="620" y1="572" x2="620" y2="688" stroke={C.panelBorder} strokeWidth="1"/>
      <text x="760" y="580" textAnchor="middle" fill={C.red} fontSize="10" fontFamily="monospace" fontWeight="bold">ÚNICO dono do LVGL</text>
      <text x="760" y="600" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">Nenhuma outra thread</text>
      <text x="760" y="616" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">pode chamar lv_*().</text>
      <text x="760" y="636" textAnchor="middle" fill={C.new} fontSize="9" fontFamily="monospace">Recebe dados via</text>
      <text x="760" y="652" textAnchor="middle" fill={C.new} fontSize="9" fontFamily="monospace">globals atômicos —</text>
      <text x="760" y="668" textAnchor="middle" fill={C.new} fontSize="9" fontFamily="monospace">sem mutex necessário.</text>

      {/* ── ARROW: main → lvgl_thread (passa ABAIXO das CAN TX boxes) ── */}
      {/* sai da base do main() em x=220, y=470; desce p/ y=500; vai dir. até x=670; sobe p/ lvgl top */}
      <Arrow d="M 220 470 L 220 500 L 670 500 L 670 510"
        col={C.new} marker="aN"
        label="g_ui_dirty / g_screen_request" lx="445" ly="494" dashed/>

      {/* ── legend ── */}
      <rect x="80" y="722" width="720" height="28" rx="3"
        fill={C.panel} stroke={C.panelBorder} strokeWidth="1"/>
      {[[100,C.new,"novo no diff"],[240,C.accent,"main thread"],
        [370,C.orange,"can TX threads"],[520,C.red,"lvgl_thread (único dono LVGL)"]].map(([x,col,lbl])=>(
        <g key={lbl}>
          <rect x={x} y={731} width="10" height="10" rx="2" fill={col}/>
          <text x={x+14} y={740} fill={C.textDim} fontSize="9" fontFamily="monospace">{lbl}</text>
        </g>
      ))}
    </svg>
  );
}

// ─── DATA FLOW ────────────────────────────────────────────────────────────────
// Layout horizontal: PRODUCERS (x 30-310) → RAM (x 360-700) → CONSUMER (x 750-1070)
// Todas as setas são horizontais ou L-bends limpos. Sem diagonais.
function DataFlowDiagram() {
  return (
    <svg viewBox="0 0 1100 720" style={{width:"100%",height:"auto"}}>
      <Defs/>
      <text x="550" y="28" textAnchor="middle" fill={C.accent}
        fontSize="15" fontFamily="monospace" fontWeight="bold" filter="url(#glow)">
        Data Flow — g_ui_dirty  (Dirty Flag Pattern)
      </text>
      <text x="550" y="46" textAnchor="middle" fill={C.textMuted}
        fontSize="10" fontFamily="monospace">
        Por que "dirty"? A tela ficou "suja" (dados mudaram) — precisa ser redesenhada.
      </text>

      {/* ══ COLUNA 1: PRODUCERS ══ */}
      <rect x="20" y="60" width="300" height="22" rx="3"
        fill="none" stroke={C.textMuted} strokeWidth="0.8" strokeDasharray="4 3"/>
      <text x="170" y="75" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">PRODUCERS (escrevem na RAM)</text>

      {/* main thread box */}
      <rect x="20" y="90" width="300" height="230" rx="6"
        fill={C.panel} stroke={C.accent} strokeWidth="1.5"/>
      <text x="170" y="112" textAnchor="middle" fill={C.accent}
        fontSize="11" fontFamily="monospace" fontWeight="bold">main thread</text>
      <text x="170" y="128" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">process_can_frame() + process_button_press()</text>
      <line x1="35" y1="136" x2="305" y2="136" stroke={C.panelBorder} strokeWidth="0.5"/>
      {[
        [152, C.textDim, "g_speed    = rpm_to_kmh(rpm)"],
        [168, C.textDim, "g_rpm      = rpm"],
        [184, C.textDim, "g_voltage  = dV"],
        [200, C.new,     "g_current  = max(U,V,W)  ← NEW"],
        [216, C.textDim, "g_direction = WIDGET_DIR_*"],
        [232, C.textDim, "g_drive_mode = pedal"],
      ].map(([y,col,txt])=>(
        <text key={y} x="36" y={y} fill={col} fontSize="9" fontFamily="monospace">{txt}</text>
      ))}
      {/* dirty flag set */}
      <rect x="36" y="248" width="268" height="26" rx="4"
        fill={C.new+"22"} stroke={C.new} strokeWidth="1.2"/>
      <text x="170" y="265" textAnchor="middle" fill={C.new}
        fontSize="10" fontFamily="monospace" fontWeight="bold">g_ui_dirty = true  ← sinaliza</text>
      <text x="170" y="300" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">(não chama LVGL diretamente!)</text>

      {/* BTN handler box */}
      <rect x="20" y="345" width="300" height="110" rx="6"
        fill={C.panel} stroke={C.orange} strokeWidth="1.2"/>
      <text x="170" y="367" textAnchor="middle" fill={C.orange}
        fontSize="11" fontFamily="monospace" fontWeight="bold">BTN6 / BTN7 handler</text>
      <text x="170" y="383" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">(roda no main thread)</text>
      <rect x="36" y="392" width="268" height="22" rx="4"
        fill={C.new+"22"} stroke={C.new} strokeWidth="1"/>
      <text x="170" y="407" textAnchor="middle" fill={C.new}
        fontSize="9" fontFamily="monospace" fontWeight="bold">g_screen_request = screen_id</text>
      <rect x="36" y="420" width="268" height="22" rx="4"
        fill={C.new+"22"} stroke={C.new} strokeWidth="1"/>
      <text x="170" y="435" textAnchor="middle" fill={C.new}
        fontSize="9" fontFamily="monospace" fontWeight="bold">g_screen_off = toggle + ioctl(FBIOSET_POWER)</text>

      {/* ══ COLUNA 2: RAM ══ */}
      <rect x="355" y="60" width="340" height="22" rx="3"
        fill="none" stroke={C.textMuted} strokeWidth="0.8" strokeDasharray="4 3"/>
      <text x="525" y="75" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">RAM — variáveis globais compartilhadas</text>

      {/* plain data globals */}
      <rect x="355" y="90" width="340" height="130" rx="5"
        fill={C.panel} stroke={C.textMuted} strokeWidth="0.8"/>
      <text x="525" y="110" textAnchor="middle" fill={C.textDim}
        fontSize="10" fontFamily="monospace" fontWeight="bold">dados brutos (int)</text>
      <text x="525" y="130" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">g_speed / g_rpm / g_voltage / g_current</text>
      <text x="525" y="148" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">g_direction / g_drive_mode</text>
      <text x="525" y="170" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">int = escrita atômica no Cortex-M7</text>
      <text x="525" y="186" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">sem mutex necessário</text>
      <text x="525" y="206" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">(pior caso: 10ms atrasado — OK para HMI)</text>

      {/* volatile dirty */}
      <rect x="355" y="234" width="340" height="44" rx="5"
        fill={C.new+"18"} stroke={C.new} strokeWidth="1.5"/>
      <text x="525" y="254" textAnchor="middle" fill={C.new}
        fontSize="11" fontFamily="monospace" fontWeight="bold">volatile bool g_ui_dirty</text>
      <text x="525" y="272" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">volatile = compilador sempre lê da RAM</text>

      {/* screen request */}
      <rect x="355" y="292" width="340" height="44" rx="5"
        fill={C.new+"18"} stroke={C.new} strokeWidth="1.5"/>
      <text x="525" y="312" textAnchor="middle" fill={C.new}
        fontSize="10" fontFamily="monospace" fontWeight="bold">volatile screen_id_t g_screen_request</text>
      <text x="525" y="330" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">SCREEN_COUNT = nenhum pedido pendente</text>

      {/* g_screen_off */}
      <rect x="355" y="350" width="340" height="44" rx="5"
        fill={C.new+"18"} stroke={C.new} strokeWidth="1.2"/>
      <text x="525" y="370" textAnchor="middle" fill={C.new}
        fontSize="10" fontFamily="monospace" fontWeight="bold">int g_screen_off</text>
      <text x="525" y="388" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">SCREEN_OFF_DISABLED / ENABLED</text>

      {/* snapshot */}
      <rect x="355" y="408" width="340" height="70" rx="5"
        fill={C.panel} stroke={C.purple} strokeWidth="1"/>
      <text x="525" y="428" textAnchor="middle" fill={C.purple}
        fontSize="10" fontFamily="monospace" fontWeight="bold">struct ui_snapshot_t  g_ui_snap</text>
      <text x="525" y="446" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">cópia local feita pela lvgl_thread</text>
      <text x="525" y="462" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">antes de chamar widgets_set_*()</text>

      {/* ══ COLUNA 3: CONSUMER ══ */}
      <rect x="750" y="60" width="320" height="22" rx="3"
        fill="none" stroke={C.textMuted} strokeWidth="0.8" strokeDasharray="4 3"/>
      <text x="910" y="75" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">CONSUMER (único dono do LVGL)</text>

      <rect x="750" y="90" width="320" height="390" rx="6"
        fill={C.panel} stroke={C.red} strokeWidth="2"/>
      <text x="910" y="116" textAnchor="middle" fill={C.red}
        fontSize="13" fontFamily="monospace" fontWeight="bold">lvgl_thread</text>
      <text x="910" y="132" textAnchor="middle" fill={C.textMuted}
        fontSize="9" fontFamily="monospace">loop a cada 10ms</text>
      <line x1="765" y1="140" x2="1055" y2="140" stroke={C.panelBorder} strokeWidth="0.5"/>
      {[
        [160, C.new,     "if (g_ui_dirty) {"],
        [178, C.textDim, "  snap = globals  // cópia segura"],
        [196, C.textDim, "  g_ui_dirty = false"],
        [214, C.textDim, "  widgets_set_speed(snap.speed)"],
        [232, C.textDim, "  widgets_set_rpm(snap.rpm)"],
        [250, C.textDim, "  widgets_set_voltage(snap.voltage)"],
        [268, C.new,     "  widgets_set_current(snap.current) ←NEW"],
        [286, C.textDim, "  widgets_set_direction(snap.dir)"],
        [304, C.textDim, "  widgets_set_mode(snap.mode)"],
        [322, C.new,     "}"],
        [348, C.new,     "if (g_screen_request ≠ SCREEN_COUNT) {"],
        [366, C.textDim, "  screen_manager_load(g_screen_request)"],
        [384, C.textDim, "  g_screen_request = SCREEN_COUNT"],
        [402, C.new,     "}"],
        [428, C.textDim, "lv_tick_inc(10)"],
        [446, C.textDim, "lv_timer_handler()"],
      ].map(([y,col,txt])=>(
        <text key={y} x="766" y={y} fill={col} fontSize="9" fontFamily="monospace">{txt}</text>
      ))}

      {/* ══ SETAS: PRODUCERS → RAM ══ */}
      {/* seta A: main → g_ui_dirty (sai de x=320, y=261, vai direto) */}
      <Arrow d="M 320 261 L 353 261" col={C.new} marker="aN"
        label="escreve globals + dirty" lx="336" ly="255"/>
      {/* seta B: BTN → g_screen_request (sai de x=320, y=407, vai p/ y=314) */}
      <Arrow d="M 320 407 L 340 407 L 340 314 L 353 314" col={C.new} marker="aN"
        label="screen_request" lx="336" ly="400"/>

      {/* ══ SETAS: RAM → CONSUMER ══ */}
      {/* seta C: g_ui_dirty → lvgl (y=256, horizontal direto) */}
      <Arrow d="M 697 256 L 748 256" col={C.new} marker="aN"
        label="lê dirty" lx="723" ly="250"/>
      {/* seta D: g_screen_request → lvgl (y=314; sem cruzar seta C) */}
      <Arrow d="M 697 314 L 748 314" col={C.new} marker="aN" dashed
        label="lê screen_req" lx="723" ly="308"/>

      {/* ══ NOTA: por que sem mutex ══ */}
      <rect x="20" y="500" width="670" height="70" rx="6"
        fill={C.panel} stroke={C.yellow} strokeWidth="1"/>
      <text x="355" y="522" textAnchor="middle" fill={C.yellow}
        fontSize="10" fontFamily="monospace" fontWeight="bold">Por que sem mutex?</text>
      <text x="355" y="542" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">
        Cortex-M7: leitura/escrita de int e bool é single-instruction → atômica.
      </text>
      <text x="355" y="558" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">
        Pior caso: lvgl_thread lê valor 10ms atrasado. Aceitável para HMI veicular.
      </text>

      {/* legend */}
      <rect x="20" y="600" width="500" height="28" rx="3"
        fill={C.panel} stroke={C.panelBorder} strokeWidth="1"/>
      {[[40,C.new,"novo/alterado"],[180,C.purple,"snapshot struct"],[310,C.red,"único dono LVGL"]].map(([x,col,lbl])=>(
        <g key={lbl}><rect x={x} y={609} width="10" height="10" rx="2" fill={col}/>
        <text x={x+14} y={618} fill={C.textDim} fontSize="9" fontFamily="monospace">{lbl}</text></g>
      ))}
    </svg>
  );
}

// ─── COMPONENT DIAGRAM ───────────────────────────────────────────────────────
// Layout em grade: 3 colunas de subsystems, main na base
// CAN (col1), Display (col2), I/O+Command (col3)
// Setas de main sobem verticalmente a um ponto central de cada subsystem
function ComponentDiagram() {
  return (
    <svg viewBox="0 0 1100 820" style={{width:"100%",height:"auto"}}>
      <Defs/>
      <text x="550" y="28" textAnchor="middle" fill={C.accent}
        fontSize="15" fontFamily="monospace" fontWeight="bold" filter="url(#glow)">
        UML 2.x Component Diagram — HMI Manager v3
      </text>
      <text x="550" y="46" textAnchor="middle" fill={C.textMuted}
        fontSize="10" fontFamily="monospace">STM32H753ZI / NuttX RTOS / CAN 2.0B @ 500kbps / LVGL 8.x</text>

      {/* system boundary */}
      <rect x="20" y="58" width="1060" height="730" rx="6"
        fill="none" stroke={C.panelBorder} strokeWidth="1.5" strokeDasharray="8 4"/>
      <text x="38" y="76" fill={C.textMuted} fontSize="10" fontFamily="monospace">
        &lt;&lt;system&gt;&gt; hmi_manager
      </text>

      {/* ── COL 1: CAN subsystem (x 30-370, y 85-430) ── */}
      <rect x="30" y="85" width="340" height="360" rx="4"
        fill="#00d4ff08" stroke={C.accentDim} strokeWidth="1.2"/>
      <text x="48" y="106" fill={C.accent} fontSize="12" fontFamily="monospace" fontWeight="bold">
        &lt;&lt;subsystem&gt;&gt; CAN
      </text>
      {/* components */}
      <rect x="50" y="118" width="140" height="52" rx="3" fill={C.panel} stroke={C.accentDim} strokeWidth="1"/>
      <text x="120" y="140" textAnchor="middle" fill={C.accent} fontSize="10" fontFamily="monospace" fontWeight="bold">can_handler</text>
      <text x="120" y="158" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">PF_CAN / SOCK_RAW</text>

      <rect x="210" y="118" width="140" height="52" rx="3" fill={C.panel} stroke={C.accentDim} strokeWidth="1"/>
      <text x="280" y="140" textAnchor="middle" fill={C.accent} fontSize="10" fontFamily="monospace" fontWeight="bold">can_trace</text>
      <text x="280" y="158" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">Frame Logger</text>

      <rect x="50" y="186" width="300" height="52" rx="3" fill={C.panel} stroke={C.accentDim} strokeWidth="1"/>
      <text x="200" y="208" textAnchor="middle" fill={C.accent} fontSize="10" fontFamily="monospace" fontWeight="bold">car_can  (DBC auto-generated)</text>
      <text x="200" y="226" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">pack() / unpack() / encode() / decode()</text>

      <rect x="50" y="254" width="140" height="50" rx="3" fill={C.panel} stroke={C.greenDim} strokeWidth="1"/>
      <text x="120" y="275" textAnchor="middle" fill={C.green} fontSize="10" fontFamily="monospace" fontWeight="bold">can_tx_100ms</text>
      <text x="120" y="291" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">pthread · 100ms</text>

      <rect x="210" y="254" width="140" height="50" rx="3" fill={C.panel} stroke={C.greenDim} strokeWidth="1"/>
      <text x="280" y="275" textAnchor="middle" fill={C.green} fontSize="10" fontFamily="monospace" fontWeight="bold">can_tx_1000ms</text>
      <text x="280" y="291" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">pthread · 1000ms</text>

      {/* NEW: RMS Currents */}
      <rect x="50" y="320" width="300" height="50" rx="3" fill={C.panel} stroke={C.new} strokeWidth="1.5"/>
      <rect x="50" y="320" width="90" height="15" rx="3" fill={C.new}/>
      <text x="95" y="331" textAnchor="middle" fill={C.bg} fontSize="8" fontFamily="monospace" fontWeight="bold">NOVO no diff</text>
      <text x="200" y="348" textAnchor="middle" fill={C.new} fontSize="10" fontFamily="monospace" fontWeight="bold">InverterRmsCurrents handler</text>
      <text x="200" y="364" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">g_current = max(phase_U, phase_V, phase_W)</text>

      {/* ── COL 2: Display subsystem (x 390-730, y 85-430) ── */}
      <rect x="390" y="85" width="340" height="360" rx="4"
        fill="#f59e0b08" stroke={C.orangeDim} strokeWidth="1.2"/>
      <text x="408" y="106" fill={C.orange} fontSize="12" fontFamily="monospace" fontWeight="bold">
        &lt;&lt;subsystem&gt;&gt; Display
      </text>
      <text x="408" y="120" fill={C.textMuted} fontSize="8" fontFamily="monospace">
        #ifdef CONFIG_HMI_MANAGER_SCREEN_ENABLE
      </text>

      <rect x="410" y="130" width="140" height="52" rx="3" fill={C.panel} stroke={C.orangeDim} strokeWidth="1"/>
      <text x="480" y="152" textAnchor="middle" fill={C.orange} fontSize="10" fontFamily="monospace" fontWeight="bold">fb_handler</text>
      <text x="480" y="170" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">/dev/fb0 + LVGL init</text>

      <rect x="570" y="130" width="140" height="52" rx="3" fill={C.panel} stroke={C.orangeDim} strokeWidth="1"/>
      <text x="640" y="152" textAnchor="middle" fill={C.orange} fontSize="10" fontFamily="monospace" fontWeight="bold">widgets</text>
      <text x="640" y="168" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">Dashboard UI</text>
      <text x="640" y="180" textAnchor="middle" fill={C.new} fontSize="8" fontFamily="monospace">+set_current() ←NEW</text>

      <rect x="410" y="198" width="140" height="52" rx="3" fill={C.panel} stroke={C.orangeDim} strokeWidth="1"/>
      <text x="480" y="220" textAnchor="middle" fill={C.orange} fontSize="10" fontFamily="monospace" fontWeight="bold">screen_manager</text>
      <text x="480" y="238" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">load() / current()</text>

      {/* lvgl_thread NEW */}
      <rect x="410" y="266" width="300" height="90" rx="3" fill={C.panel} stroke={C.new} strokeWidth="2"/>
      <rect x="410" y="254" width="110" height="16" rx="3" fill={C.new}/>
      <text x="465" y="266" textAnchor="middle" fill={C.bg} fontSize="8" fontFamily="monospace" fontWeight="bold">NOVO no diff</text>
      <text x="560" y="292" textAnchor="middle" fill={C.new} fontSize="11" fontFamily="monospace" fontWeight="bold">lvgl_thread</text>
      <text x="560" y="310" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">único caller de LVGL</text>
      <text x="560" y="326" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">lê g_ui_dirty + g_screen_request</text>
      <text x="560" y="342" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">prio: DEFAULT-10  ·  stack: 4096B</text>

      {/* ── COL 3: I/O + Command (x 750-1060, y 85-430) ── */}
      {/* I/O */}
      <rect x="750" y="85" width="300" height="205" rx="4"
        fill="#10b98108" stroke={C.greenDim} strokeWidth="1.2"/>
      <text x="768" y="106" fill={C.green} fontSize="12" fontFamily="monospace" fontWeight="bold">
        &lt;&lt;subsystem&gt;&gt; I/O
      </text>

      <rect x="768" y="118" width="120" height="50" rx="3" fill={C.panel} stroke={C.greenDim} strokeWidth="1"/>
      <text x="828" y="140" textAnchor="middle" fill={C.green} fontSize="10" fontFamily="monospace" fontWeight="bold">button_irq</text>
      <text x="828" y="158" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">/dev/buttons</text>

      <rect x="910" y="118" width="120" height="50" rx="3" fill={C.panel} stroke={C.greenDim} strokeWidth="1"/>
      <text x="970" y="140" textAnchor="middle" fill={C.green} fontSize="10" fontFamily="monospace" fontWeight="bold">led_control</text>
      <text x="970" y="158" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">/dev/userleds</text>

      {/* BTN7 NEW */}
      <rect x="768" y="184" width="262" height="88" rx="3" fill={C.panel} stroke={C.new} strokeWidth="1.5"/>
      <rect x="768" y="172" width="100" height="15" rx="3" fill={C.new}/>
      <text x="818" y="183" textAnchor="middle" fill={C.bg} fontSize="8" fontFamily="monospace" fontWeight="bold">NOVO no diff</text>
      <text x="899" y="208" textAnchor="middle" fill={C.new} fontSize="10" fontFamily="monospace" fontWeight="bold">BTN7 toggle screen off</text>
      <text x="899" y="226" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">g_screen_off flag</text>
      <text x="899" y="242" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">ioctl(fb0, FBIOSET_POWER, 0/1)</text>
      <text x="899" y="258" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">restaura tela via g_screen_request</text>

      {/* Command */}
      <rect x="750" y="308" width="300" height="137" rx="4"
        fill="#a78bfa08" stroke={C.purpleDim} strokeWidth="1.2"/>
      <text x="768" y="328" fill={C.purple} fontSize="12" fontFamily="monospace" fontWeight="bold">
        &lt;&lt;subsystem&gt;&gt; Command
      </text>

      <rect x="768" y="340" width="120" height="50" rx="3" fill={C.panel} stroke={C.purpleDim} strokeWidth="1"/>
      <text x="828" y="362" textAnchor="middle" fill={C.purple} fontSize="10" fontFamily="monospace" fontWeight="bold">commands</text>
      <text x="828" y="378" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">argc/argv router</text>

      <rect x="910" y="340" width="120" height="50" rx="3" fill={C.panel} stroke={C.purpleDim} strokeWidth="1"/>
      <text x="970" y="362" textAnchor="middle" fill={C.purple} fontSize="10" fontFamily="monospace" fontWeight="bold">hmi_ctl</text>
      <text x="970" y="378" textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">AF_UNIX client</text>

      {/* ── main() Orchestrator ── */}
      <rect x="100" y="465" width="860" height="65" rx="4"
        fill={C.panel} stroke={C.accent} strokeWidth="1.5"/>
      <text x="530" y="490" textAnchor="middle" fill={C.accent}
        fontSize="13" fontFamily="monospace" fontWeight="bold" filter="url(#glow)">
        main() — Orchestrator
      </text>
      <text x="530" y="508" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">init all → spawn 4 threads → select(cmd_socket, 10ms) + poll buttons</text>
      <text x="530" y="522" textAnchor="middle" fill={C.new}
        fontSize="9" fontFamily="monospace">escreve globals + g_ui_dirty / g_screen_request  (não toca LVGL diretamente)</text>

      {/* ── setas de main para subsystems (todas verticais, ponto de saída diferente) ── */}
      {/* → CAN: sobe de x=200 */}
      <Arrow d="M 200 465 L 200 447" col={C.accentDim} marker="aC" dashed/>
      {/* → Display/lvgl: sobe de x=560 */}
      <Arrow d="M 560 465 L 560 358" col={C.new} marker="aN" dashed/>
      {/* → I/O: L-bend: sobe de x=760 até y=450, depois sobe direto */}
      <Arrow d="M 760 465 L 760 293" col={C.greenDim} marker="aG" dashed/>
      {/* → Command: sobe de x=900 */}
      <Arrow d="M 860 465 L 860 447" col={C.purpleDim} marker="aP" dashed/>

      {/* ── external devices ── */}
      {[
        [30,  570, 200, "FDCAN1  /dev/can0",  "CAN 2.0B @ 500kbps"],
        [390, 570, 200, "ST7796  /dev/fb0",   "SPI 480×320 RGB565"],
        [750, 570, 140, "11× GPIO Buttons",   "IRQ + debounce"],
        [910, 570, 140, "3× User LEDs",       "G / O / R"],
      ].map(([x,y,w,l1,l2])=>(
        <g key={l1}>
          <rect x={x} y={y} width={w} height={52} rx="3"
            fill="none" stroke={C.textMuted} strokeWidth="1" strokeDasharray="4 3"/>
          <text x={x+w/2} y={y+20} textAnchor="middle" fill={C.textMuted} fontSize="9" fontFamily="monospace">{l1}</text>
          <text x={x+w/2} y={y+38} textAnchor="middle" fill={C.textMuted} fontSize="8" fontFamily="monospace">{l2}</text>
          <line x1={x+w/2} y1={y} x2={x+w/2} y2={y-10} stroke={C.textMuted} strokeWidth="0.8" strokeDasharray="3 3"/>
        </g>
      ))}

      {/* legend */}
      <rect x="30" y="648" width="1010" height="28" rx="3"
        fill={C.panel} stroke={C.panelBorder} strokeWidth="1"/>
      {[[50,C.accent,"CAN"],[130,C.orange,"Display"],[230,C.green,"I/O"],
        [310,C.purple,"Command"],[430,C.new,"novo no diff"]].map(([x,col,lbl])=>(
        <g key={lbl}><circle cx={x} cy={662} r="5" fill={col}/>
        <text x={x+10} y={666} fill={C.textDim} fontSize="9" fontFamily="monospace">{lbl}</text></g>
      ))}
    </svg>
  );
}

// ─── SEQUENCE DIAGRAM ────────────────────────────────────────────────────────
function SequenceDiagram() {
  // lifelines: NSH | main | can_tx | lvgl | can_frame | btn | hmi_ctl
  const lx = [80, 220, 370, 510, 650, 790, 940];
  const names = ["NSH/rcS","main()","can_tx\nthreads","lvgl_\nthread","process\n_can","btn\nhandler","hmi_ctl"];
  const cols  = [C.textMuted,C.accent,C.orange,C.new,C.green,C.green,C.purple];

  function Msg({x1,y1,x2,y2,col,lbl,dashed=false,above=true}) {
    const mx = (x1+x2)/2, my = above ? Math.min(y1,y2)-7 : Math.max(y1,y2)+13;
    return (
      <g>
        <line x1={x1} y1={y1} x2={x2} y2={y2} stroke={col} strokeWidth="1.2"
          strokeDasharray={dashed?"5 3":undefined} markerEnd={`url(#${col===C.new?"aN":col===C.orange?"aO":col===C.green?"aG":col===C.purple?"aP":"aC"})`}/>
        {lbl && <text x={mx} y={my} fill={col} fontSize="8" fontFamily="monospace" textAnchor="middle">{lbl}</text>}
      </g>
    );
  }

  return (
    <svg viewBox="0 0 1060 900" style={{width:"100%",height:"auto"}}>
      <Defs/>
      <text x="530" y="28" textAnchor="middle" fill={C.accent}
        fontSize="15" fontFamily="monospace" fontWeight="bold">
        UML 2.x Sequence — Boot &amp; Runtime v3
      </text>

      {/* lifelines */}
      {lx.map((x,i)=>(
        <g key={i}>
          <rect x={x-46} y={44} width={92} height={34} rx="3"
            fill={C.panel} stroke={cols[i]} strokeWidth="1.2"/>
          <text x={x} y={58} textAnchor="middle" fill={cols[i]} fontSize="9" fontFamily="monospace" fontWeight="bold">
            {names[i].split("\n")[0]}
          </text>
          {names[i].includes("\n") && (
            <text x={x} y={70} textAnchor="middle" fill={cols[i]} fontSize="9" fontFamily="monospace" fontWeight="bold">
              {names[i].split("\n")[1]}
            </text>
          )}
          <line x1={x} y1={78} x2={x} y2={882} stroke={cols[i]} strokeWidth="0.5" strokeDasharray="4 4"/>
        </g>
      ))}

      {/* activation bars */}
      <rect x={lx[0]-5} y={92}  width={10} height={32}  fill={C.textMuted} opacity="0.25"/>
      <rect x={lx[1]-5} y={125} width={10} height={600} fill={C.accent}    opacity="0.10"/>
      <rect x={lx[2]-5} y={360} width={10} height={522} fill={C.orange}    opacity="0.10"/>
      <rect x={lx[3]-5} y={385} width={10} height={497} fill={C.new}       opacity="0.13"/>
      <rect x={lx[4]-5} y={410} width={10} height={472} fill={C.green}     opacity="0.10"/>

      {/* ── BOOT ── */}
      <Msg x1={lx[0]} y1={105} x2={lx[1]} y2={105} col={C.textMuted} lbl="hmi_manager &"/>

      {/* ── INIT (self-calls on main) ── */}
      {[
        [140,"fb_handler_init()"],
        [165,"led_control_init()"],
        [190,"can_handler_init()"],
        [215,"button_irq_init()"],
        [240,"init_command_socket()"],
        [265,"widgets_init()"],
      ].map(([y,lbl])=>(
        <g key={y}>
          <path d={`M${lx[1]+5},${y} L${lx[1]+55},${y} L${lx[1]+55},${y+14} L${lx[1]+5},${y+14}`}
            fill="none" stroke={C.accentDim} strokeWidth="0.8" markerEnd="url(#aC)"/>
          <text x={lx[1]+62} y={y+10} fill={C.textDim} fontSize="8" fontFamily="monospace">{lbl}</text>
        </g>
      ))}

      {/* ── SPAWN ── */}
      <rect x={lx[1]-50} y={300} width={170} height={20} rx="2"
        fill={C.panel} stroke={C.accent} strokeWidth="0.8"/>
      <text x={lx[1]+35} y={314} textAnchor="middle" fill={C.accent}
        fontSize="8" fontFamily="monospace">spawn 4 pthreads</text>

      {/* spawn arrows — um por linha, labels à esquerda */}
      <Msg x1={lx[1]} y1={332} x2={lx[2]} y2={332} col={C.orange} lbl="can_tx x2"/>
      <Msg x1={lx[1]} y1={357} x2={lx[3]} y2={357} col={C.new}    lbl="lvgl_thread (NOVO)"/>
      <Msg x1={lx[1]} y1={382} x2={lx[4]} y2={382} col={C.green}  lbl="process_can context"/>

      {/* ── LVGL loop box ── */}
      <rect x={lx[3]-52} y={430} width={104} height={70} rx="2"
        fill="none" stroke={C.new} strokeWidth="1"/>
      <text x={lx[3]} y={448} textAnchor="middle" fill={C.new} fontSize="8" fontFamily="monospace">loop 10ms</text>
      <text x={lx[3]} y={464} textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">check g_ui_dirty</text>
      <text x={lx[3]} y={480} textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">check g_screen_req</text>
      <text x={lx[3]} y={496} textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">lv_timer_handler</text>

      {/* ── CAN frame → main sets dirty ── */}
      <Msg x1={lx[4]} y1={560} x2={lx[1]} y2={560} col={C.green} lbl="CAN frame decoded"/>
      {/* self note on main */}
      <path d={`M${lx[1]+5},580 L${lx[1]+70},580 L${lx[1]+70},610 L${lx[1]+5},610`}
        fill="none" stroke={C.accentDim} strokeWidth="0.8" markerEnd="url(#aC)"/>
      <text x={lx[1]+76} y={592} fill={C.new} fontSize="8" fontFamily="monospace">g_speed/rpm/voltage/current = …</text>
      <text x={lx[1]+76} y={607} fill={C.new} fontSize="8" fontFamily="monospace">g_ui_dirty = true  (NÃO chama widgets!)</text>

      {/* ── lvgl reads dirty → renders ── */}
      <path d={`M${lx[3]+5},645 L${lx[3]+60},645 L${lx[3]+60},668 L${lx[3]+5},668`}
        fill="none" stroke={C.new} strokeWidth="0.9" markerEnd="url(#aN)"/>
      <text x={lx[3]+66} y={658} fill={C.new} fontSize="8" fontFamily="monospace">snapshot + widgets_set_*() — seguro!</text>

      {/* ── BTN → screen request ── */}
      <Msg x1={lx[5]} y1={710} x2={lx[1]} y2={710} col={C.green} lbl="BTN6/BTN7 pressed"/>
      <path d={`M${lx[1]+5},728 L${lx[1]+70},728 L${lx[1]+70},748 L${lx[1]+5},748`}
        fill="none" stroke={C.accentDim} strokeWidth="0.8" markerEnd="url(#aC)"/>
      <text x={lx[1]+76} y={740} fill={C.new} fontSize="8" fontFamily="monospace">g_screen_request = id + ioctl(FBIOSET_POWER)</text>

      {/* ── lvgl reads screen request ── */}
      <path d={`M${lx[3]+5},775 L${lx[3]+60},775 L${lx[3]+60},795 L${lx[3]+5},795`}
        fill="none" stroke={C.new} strokeWidth="0.9" markerEnd="url(#aN)"/>
      <text x={lx[3]+66} y={788} fill={C.new} fontSize="8" fontFamily="monospace">screen_manager_load(req) — seguro!</text>

      {/* ── hmi_ctl command ── */}
      <Msg x1={lx[6]} y1={840} x2={lx[1]} y2={840} col={C.purple} lbl="AF_UNIX: can --trace show"/>
      <path d={`M${lx[1]+5},854 L${lx[1]+55},854 L${lx[1]+55},868 L${lx[1]+5},868`}
        fill="none" stroke={C.purpleDim} strokeWidth="0.8" markerEnd="url(#aP)"/>
      <text x={lx[1]+62} y={864} fill={C.purple} fontSize="8" fontFamily="monospace">commands_parse()</text>

      <text x="530" y="890" textAnchor="middle" fill={C.textMuted} fontSize="9" fontFamily="monospace">
        Todas as threads NuttX independentes · LVGL tocado apenas por lvgl_thread
      </text>
    </svg>
  );
}

// ─── STATE MACHINE ───────────────────────────────────────────────────────────
function StateDiagram() {
  return (
    <svg viewBox="0 0 1100 700" style={{width:"100%",height:"auto"}}>
      <Defs/>
      <defs>
        <marker id="sA" viewBox="0 0 10 7" refX="9" refY="3.5"
          markerWidth="7" markerHeight="5" orient="auto-start-reverse">
          <polygon points="0 0,10 3.5,0 7" fill={C.textDim}/>
        </marker>
      </defs>
      <text x="550" y="28" textAnchor="middle" fill={C.accent}
        fontSize="15" fontFamily="monospace" fontWeight="bold">
        UML 2.x State Machine — HMI Manager Lifecycle v3
      </text>

      {/* initial dot */}
      <circle cx="130" cy="72" r="11" fill={C.accent}/>

      {/* BOOTING */}
      <rect x="65" y="100" width="190" height="60" rx="18"
        fill={C.panel} stroke={C.accent} strokeWidth="1.5"/>
      <text x="160" y="124" textAnchor="middle" fill={C.accent} fontSize="12" fontFamily="monospace" fontWeight="bold">BOOTING</text>
      <text x="160" y="142" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">rcS → hmi_manager &amp;</text>
      <line x1="130" y1="83" x2="130" y2="100" stroke={C.textDim} strokeWidth="1" markerEnd="url(#sA)"/>

      {/* INITIALIZING */}
      <rect x="65" y="210" width="190" height="130" rx="18"
        fill={C.panel} stroke={C.orange} strokeWidth="1.5"/>
      <text x="160" y="234" textAnchor="middle" fill={C.orange} fontSize="12" fontFamily="monospace" fontWeight="bold">INITIALIZING</text>
      <text x="160" y="252" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">entry/ fb_handler_init</text>
      <text x="160" y="268" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">do/ led, CAN, btn, cmd</text>
      <text x="160" y="284" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">do/ widgets_init</text>
      <text x="160" y="300" textAnchor="middle" fill={C.new} fontSize="9" fontFamily="monospace">exit/ spawn 4 pthreads</text>
      <line x1="160" y1="160" x2="160" y2="210" stroke={C.textDim} strokeWidth="1" markerEnd="url(#sA)"/>

      {/* INIT_FAILED */}
      <rect x="65" y="420" width="190" height="55" rx="18"
        fill={C.panel} stroke={C.red} strokeWidth="1.5"/>
      <text x="160" y="446" textAnchor="middle" fill={C.red} fontSize="12" fontFamily="monospace" fontWeight="bold">INIT_FAILED</text>
      <text x="160" y="464" textAnchor="middle" fill={C.textDim} fontSize="9" fontFamily="monospace">entry/ return 1</text>
      <line x1="160" y1="340" x2="160" y2="420" stroke={C.red} strokeWidth="1" markerEnd="url(#sA)"/>
      <text x="176" y="385" fill={C.red} fontSize="9" fontFamily="monospace">[fatal error]</text>
      {/* final */}
      <circle cx="160" cy="530" r="9" fill="none" stroke={C.textDim} strokeWidth="1.5"/>
      <circle cx="160" cy="530" r="5" fill={C.textDim}/>
      <line x1="160" y1="475" x2="160" y2="521" stroke={C.textDim} strokeWidth="1" markerEnd="url(#sA)"/>

      {/* RUNNING composite */}
      <rect x="305" y="62" width="775" height="460" rx="8"
        fill="none" stroke={C.green} strokeWidth="2"/>
      <text x="692" y="84" textAnchor="middle" fill={C.green}
        fontSize="13" fontFamily="monospace" fontWeight="bold">RUNNING (composite — orthogonal regions)</text>

      {/* row 1 sub-states */}
      {[
        [320,  100, C.green,  "CAN_RX",     "do/ blocking read()", "do/ process_can_frame"],
        [510,  100, C.orange, "CAN_TX",      "do/ pack + write()",  "do/ sleep(period)"],
        [700,  100, C.new,    "RENDERING",  "do/ check g_ui_dirty","do/ lv_timer_handler  NEW"],
        [890,  100, C.new,    "SCREEN_MGR", "do/ check g_screen_req","do/ screen_manager_load  NEW"],
      ].map(([x,y,col,t,s1,s2])=>(
        <g key={t}>
          <rect x={x} y={y} width={170} height={88} rx="12"
            fill={C.panel} stroke={col} strokeWidth={col===C.new?1.8:1.2}/>
          <text x={x+85} y={y+22} textAnchor="middle" fill={col} fontSize="10" fontFamily="monospace" fontWeight="bold">{t}</text>
          <text x={x+85} y={y+42} textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">{s1}</text>
          <text x={x+85} y={y+58} textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">{s2.replace("  NEW","")}</text>
          {s2.includes("NEW") && <text x={x+85} y={y+74} textAnchor="middle" fill={C.new} fontSize="8" fontFamily="monospace">← NEW thread</text>}
        </g>
      ))}

      {/* row 2 sub-states */}
      {[
        [320, 216, C.purple, "CMD_LISTEN", "do/ select(cmd_fd)",       "do/ commands_parse"],
        [510, 216, C.green,  "BTN_POLL",   "do/ get_pressed()",        "do/ process_button"],
        [700, 216, C.new,    "SCREEN_OFF", "do/ ioctl(FBIOSET_POWER,0)","g_screen_off=ENABLED  NEW"],
      ].map(([x,y,col,t,s1,s2])=>(
        <g key={t}>
          <rect x={x} y={y} width={170} height={88} rx="12"
            fill={C.panel} stroke={col} strokeWidth={col===C.new?1.8:1.2}/>
          <text x={x+85} y={y+22} textAnchor="middle" fill={col} fontSize="10" fontFamily="monospace" fontWeight="bold">{t}</text>
          <text x={x+85} y={y+42} textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">{s1}</text>
          <text x={x+85} y={y+58} textAnchor="middle" fill={C.textDim} fontSize="8" fontFamily="monospace">{s2.replace("  NEW","")}</text>
          {s2.includes("NEW") && <text x={x+85} y={y+74} textAnchor="middle" fill={C.new} fontSize="8" fontFamily="monospace">← NEW state</text>}
        </g>
      ))}

      {/* orthogonal region separators */}
      <line x1="315" y1="205" x2="1060" y2="205" stroke={C.green} strokeWidth="0.5" strokeDasharray="6 3"/>
      <line x1="506" y1="95"  x2="506"  y2="310" stroke={C.green} strokeWidth="0.5" strokeDasharray="6 3"/>
      <line x1="696" y1="95"  x2="696"  y2="310" stroke={C.green} strokeWidth="0.5" strokeDasharray="6 3"/>
      <line x1="886" y1="95"  x2="886"  y2="205" stroke={C.green} strokeWidth="0.5" strokeDasharray="6 3"/>

      {/* DC Link sub-state */}
      <rect x="320" y="348" width="680" height="100" rx="4"
        fill={C.panel} stroke={C.accentDim} strokeWidth="1"/>
      <text x="660" y="368" textAnchor="middle" fill={C.accent}
        fontSize="10" fontFamily="monospace" fontWeight="bold">DC Link Sub-state  (driven by CAN RX: INVERTER_INFO)</text>
      {[[350,"OFF",C.textMuted],[520,"PRE_CHARGE",C.orange],[690,"ON",C.green]].map(([x,lbl,col])=>(
        <g key={lbl}>
          <rect x={x} y={384} width={130} height={38} rx="10" fill="none" stroke={col} strokeWidth="1"/>
          <text x={x+65} y={407} textAnchor="middle" fill={col} fontSize="10" fontFamily="monospace">{lbl}</text>
        </g>
      ))}
      <line x1="480" y1="403" x2="520" y2="403" stroke={C.textDim} strokeWidth="1" markerEnd="url(#sA)"/>
      <line x1="650" y1="403" x2="690" y2="403" stroke={C.textDim} strokeWidth="1" markerEnd="url(#sA)"/>
      <path d="M 690 416 C 630 448 430 448 350 416" fill="none"
        stroke={C.red} strokeWidth="0.8" strokeDasharray="4 3" markerEnd="url(#sA)"/>
      <text x="520" y="452" textAnchor="middle" fill={C.red} fontSize="8" fontFamily="monospace">[fault / demand off]</text>

      {/* BTN carousel note */}
      <rect x="320" y="468" width="680" height="42" rx="4"
        fill={C.panel} stroke={C.new} strokeWidth="1.2"/>
      <text x="660" y="486" textAnchor="middle" fill={C.new}
        fontSize="9" fontFamily="monospace" fontWeight="bold">BTN6 Carousel: DASHBOARD → THEME → repeat  (SCREEN_SPLASH removido)</text>
      <text x="660" y="502" textAnchor="middle" fill={C.textDim}
        fontSize="9" fontFamily="monospace">BTN7 toggle screen off · switch sempre via g_screen_request → lvgl_thread (thread-safe)</text>

      {/* INIT → RUNNING transition */}
      <line x1="255" y1="275" x2="305" y2="220" stroke={C.textDim} strokeWidth="1.2" markerEnd="url(#sA)"/>
      <text x="258" y="248" fill={C.textDim} fontSize="9" fontFamily="monospace">[all init OK]</text>

      <text x="550" y="688" textAnchor="middle" fill={C.textMuted} fontSize="9" fontFamily="monospace">
        UML 2.x Composite State · Orthogonal Regions · v3 pós-diff · Vinicius R. May
      </text>
    </svg>
  );
}

// ─── APP ─────────────────────────────────────────────────────────────────────
export default function App() {
  const [active, setActive] = useState("thread");
  const views = {thread:ThreadDiagram, dataflow:DataFlowDiagram,
                 component:ComponentDiagram, sequence:SequenceDiagram, state:StateDiagram};
  const View = views[active];
  return (
    <div style={{background:C.bg, minHeight:"100vh", padding:"16px",
                 fontFamily:"'JetBrains Mono','Fira Code',monospace"}}>
      <div style={{maxWidth:1100, margin:"0 auto"}}>
        <div style={{textAlign:"center", marginBottom:16,
                     borderBottom:`1px solid ${C.panelBorder}`, paddingBottom:12}}>
          <h1 style={{color:C.accent, fontSize:18, margin:0,
                      letterSpacing:2, textTransform:"uppercase"}}>HMI Manager — UML 2.x</h1>
          <p style={{color:C.new, fontSize:10, margin:"4px 0 0"}}>
            v4 · pós-diff · STM32H753ZI | NuttX | CAN 2.0B | LVGL 8.x
            <span style={{color:C.textMuted, marginLeft:12}}>itens verdes = mudanças do diff</span>
          </p>
        </div>
        <div style={{display:"flex", gap:3, marginBottom:14,
                     background:C.panel, borderRadius:6, padding:3,
                     border:`1px solid ${C.panelBorder}`}}>
          {tabs.map(t=>(
            <button key={t.id} onClick={()=>setActive(t.id)} style={{
              flex:1, padding:"8px 0", border:"none", borderRadius:4, cursor:"pointer",
              fontSize:11, fontFamily:"inherit",
              fontWeight: active===t.id ? "bold" : "normal",
              background: active===t.id ? (t.id==="dataflow"?C.new+"22":C.accent+"22") : "transparent",
              color: active===t.id ? (t.id==="dataflow"?C.new:C.accent) : C.textMuted,
              borderBottom: active===t.id ? `2px solid ${t.id==="dataflow"?C.new:C.accent}` : "2px solid transparent",
              transition:"all 0.2s",
            }}>{t.label}</button>
          ))}
        </div>
        <div style={{background:C.panel, border:`1px solid ${C.panelBorder}`,
                     borderRadius:8, padding:12, overflow:"auto"}}>
          <View/>
        </div>
        <div style={{textAlign:"center", marginTop:10, color:C.textMuted, fontSize:10}}>
          UML 2.x Compliant · v4.0 March 2026 · Vinicius R. May
        </div>
      </div>
    </div>
  );
}
