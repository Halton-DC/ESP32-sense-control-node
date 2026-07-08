/**
 * webui.h - Embedded single-page settings UI (served over HTTPS).
 * Self-contained: no external fonts/scripts (works fully offline / air-gapped).
 * Design language: iOS/macOS System Settings - SF system type, grouped inset
 * rows, frosted chrome, layered depth, spring motion, light + dark.
 */
#pragma once
#include <Arduino.h>

static const char WEBUI_HTML[] PROGMEM = R"HTMLDOC(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"/>
<meta name="theme-color" content="#f2f2f7" media="(prefers-color-scheme: light)"/>
<meta name="theme-color" content="#000000" media="(prefers-color-scheme: dark)"/>
<title>HDC Node</title>
<style>
:root{
  --bg:#f2f2f7; --group:#ffffff; --tile:#ffffff; --chrome:rgba(249,249,251,.72);
  --label:#1c1c1e; --label2:rgba(60,60,67,.62); --label3:rgba(60,60,67,.36);
  --sep:rgba(60,60,67,.14); --hair:rgba(60,60,67,.10);
  --accent:#0d9488; --accent-press:#0b7d73; --on:#34c759; --danger:#ff3b30;
  --field:#ffffff; --field-line:rgba(60,60,67,.20);
  --shadow:0 .5px 1px rgba(0,0,0,.04),0 8px 28px rgba(0,0,0,.07);
  --radius:18px; --ctl:12px;
  --spring:cubic-bezier(.32,.72,0,1);
}
@media (prefers-color-scheme:dark){
  :root{
    --bg:#000000; --group:#1c1c1e; --tile:#1c1c1e; --chrome:rgba(24,24,26,.72);
    --label:#f5f5f7; --label2:rgba(235,235,245,.60); --label3:rgba(235,235,245,.30);
    --sep:rgba(84,84,88,.55); --hair:rgba(84,84,88,.4);
    --accent:#2dd4bf; --accent-press:#5ee0d0; --on:#30d158; --danger:#ff453a;
    --field:#2c2c2e; --field-line:rgba(84,84,88,.6);
    --shadow:0 .5px 1px rgba(0,0,0,.5),0 10px 30px rgba(0,0,0,.5);
  }
}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0}
body{background:var(--bg);color:var(--label);
  font:400 16px/1.45 -apple-system,BlinkMacSystemFont,"SF Pro Text","Segoe UI",Roboto,Helvetica,Arial,sans-serif;
  -webkit-font-smoothing:antialiased;text-rendering:optimizeLegibility;
  padding-top:env(safe-area-inset-top);}
h1,h2,h3{margin:0;letter-spacing:-.021em}
button,input{font:inherit;color:inherit}
button{cursor:pointer;border:0;background:none}
.hidden{display:none!important}
.mono{font-variant-numeric:tabular-nums;font-feature-settings:"tnum"}
::selection{background:color-mix(in srgb,var(--accent) 30%,transparent)}

/* Chrome */
header{position:sticky;top:0;z-index:20;display:flex;align-items:center;gap:14px;
  padding:12px max(20px,env(safe-area-inset-left)) 12px max(20px,env(safe-area-inset-left));
  background:var(--chrome);backdrop-filter:saturate(180%) blur(20px);-webkit-backdrop-filter:saturate(180%) blur(20px);
  border-bottom:.5px solid var(--sep)}
.brand{width:30px;height:30px;border-radius:8px;flex:0 0 auto;
  background:linear-gradient(160deg,var(--accent),color-mix(in srgb,var(--accent) 55%,#000));
  color:#fff;display:grid;place-items:center;font-weight:800;font-size:15px;box-shadow:var(--shadow)}
.hgroup .n{font-weight:700;font-size:17px;letter-spacing:-.02em;line-height:1.15}
.hgroup .l{font-size:12.5px;color:var(--label2)}
.grow{flex:1}
.chip{display:inline-flex;align-items:center;gap:7px;font-size:12.5px;color:var(--label2);
  padding:6px 11px;border-radius:999px;background:var(--group);border:.5px solid var(--sep)}
.dot{width:7px;height:7px;border-radius:50%;background:var(--label3);box-shadow:0 0 0 0 transparent}
.dot.on{background:var(--on);box-shadow:0 0 0 3px color-mix(in srgb,var(--on) 22%,transparent)}
.dot.off{background:var(--danger)}
.txtbtn{color:var(--accent);font-weight:600;font-size:15px;padding:6px 4px;transition:opacity .2s}
.txtbtn:active{opacity:.4}

/* Layout */
.wrap{max-width:940px;margin:0 auto;display:grid;grid-template-columns:220px 1fr;gap:26px;
  padding:26px max(20px,env(safe-area-inset-left)) 80px}
nav{position:sticky;top:78px;align-self:start;display:flex;flex-direction:column;gap:2px}
nav button{display:flex;align-items:center;gap:12px;width:100%;padding:9px 12px;border-radius:10px;
  color:var(--label);font-size:15px;font-weight:500;text-align:left;transition:background .18s var(--spring)}
nav button svg{width:19px;height:19px;color:var(--label2);flex:0 0 auto}
nav button:hover{background:var(--hair)}
nav button.active{background:var(--group);box-shadow:var(--shadow);font-weight:600}
nav button.active svg{color:var(--accent)}
@media(max-width:720px){
  .wrap{grid-template-columns:1fr;gap:18px;padding-top:16px}
  nav{position:static;flex-direction:row;overflow-x:auto;gap:6px;padding-bottom:2px;scrollbar-width:none}
  nav::-webkit-scrollbar{display:none}
  nav button{white-space:nowrap;padding:8px 14px;border:.5px solid var(--sep)}
  nav button span{display:inline}
}

/* Panes + groups */
section{animation:fade .4s var(--spring)}
@keyframes fade{from{opacity:0;transform:translateY(6px)}to{opacity:1;transform:none}}
.gtitle{font-size:13px;font-weight:600;letter-spacing:.02em;text-transform:uppercase;color:var(--label2);
  margin:22px 4px 8px}
.gtitle:first-child{margin-top:0}
.group{background:var(--group);border-radius:var(--radius);box-shadow:var(--shadow);overflow:hidden}
.gnote{color:var(--label2);font-size:13px;margin:8px 6px 0;line-height:1.4}
.row{display:flex;align-items:center;gap:14px;padding:13px 16px;min-height:52px;position:relative}
.row+.row::before{content:"";position:absolute;top:0;left:16px;right:0;height:.5px;background:var(--sep)}
.row .k{font-size:16px;font-weight:500}
.row .sub{font-size:12.5px;color:var(--label2);margin-top:1px}
.row .v{margin-left:auto;color:var(--label2);font-size:16px;display:flex;align-items:center;gap:8px}
.row.tap{cursor:pointer;transition:background .12s}
.row.tap:active{background:var(--hair)}
.row.col{flex-direction:column;align-items:stretch;gap:8px}

/* Inputs */
input[type=text],input[type=password],input[type=number]{width:100%;background:var(--field);
  border:.5px solid var(--field-line);border-radius:var(--ctl);padding:11px 13px;font-size:16px;
  transition:border-color .2s,box-shadow .2s;-webkit-appearance:none}
input:focus{outline:none;border-color:var(--accent);box-shadow:0 0 0 4px color-mix(in srgb,var(--accent) 18%,transparent)}
.inline-input{margin-left:auto;max-width:56%;text-align:right;border:0;background:transparent;padding:0;font-size:16px;color:var(--label2)}
.inline-input:focus{box-shadow:none;color:var(--label)}
label.fld{display:block;padding:12px 16px}
label.fld .cap{font-size:12.5px;color:var(--label2);margin-bottom:6px;display:block}

/* Toggle (iOS) */
.sw{position:relative;width:51px;height:31px;flex:0 0 auto;margin-left:auto}
.sw input{position:absolute;opacity:0;width:100%;height:100%;margin:0;cursor:pointer;z-index:2}
.sw .tk{position:absolute;inset:0;background:var(--field-line);border-radius:999px;transition:background .3s var(--spring)}
.sw .tk::after{content:"";position:absolute;top:2px;left:2px;width:27px;height:27px;border-radius:50%;background:#fff;
  box-shadow:0 3px 8px rgba(0,0,0,.15),0 1px 1px rgba(0,0,0,.16);transition:transform .3s var(--spring)}
.sw input:checked+.tk{background:var(--on)}
.sw input:checked+.tk::after{transform:translateX(20px)}

/* Buttons */
.btn{display:inline-flex;align-items:center;justify-content:center;gap:8px;width:100%;
  background:var(--accent);color:#fff;font-weight:600;font-size:16px;padding:13px 18px;border-radius:14px;
  transition:transform .15s var(--spring),filter .2s,background .2s}
.btn:hover{filter:brightness(1.04)}
.btn:active{transform:scale(.975);background:var(--accent-press)}
.btn.tinted{background:color-mix(in srgb,var(--accent) 14%,transparent);color:var(--accent)}
.btn.danger{background:color-mix(in srgb,var(--danger) 14%,transparent);color:var(--danger)}
.btn.danger:active{background:color-mix(in srgb,var(--danger) 22%,transparent)}
.pad{padding:14px 16px}
.saverow{padding:14px 0 2px}

/* Dashboard hero */
.hero{background:var(--group);border-radius:var(--radius);box-shadow:var(--shadow);padding:22px 24px;
  display:flex;align-items:flex-end;justify-content:space-between;gap:18px;flex-wrap:wrap}
.hero .big{font-size:66px;font-weight:600;letter-spacing:-.04em;line-height:.9}
.hero .big .deg{font-size:34px;color:var(--label2);font-weight:500;vertical-align:top}
.hero .cap{color:var(--label2);font-size:14px;margin-top:8px}
.hero .side{display:flex;gap:26px}
.hero .side .s .l{font-size:12px;color:var(--label2);text-transform:uppercase;letter-spacing:.04em}
.hero .side .s .val{font-size:24px;font-weight:600;letter-spacing:-.02em;margin-top:3px}
.tiles{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:12px;margin-top:12px}
.tile{background:var(--tile);border-radius:16px;box-shadow:var(--shadow);padding:15px 16px}
.tile .l{font-size:12px;color:var(--label2);text-transform:uppercase;letter-spacing:.04em;display:flex;align-items:center;gap:6px}
.tile .val{font-size:26px;font-weight:600;letter-spacing:-.02em;margin-top:6px}
.tile .u{font-size:14px;color:var(--label2);font-weight:500}
.badge{font-size:12px;font-weight:600;padding:3px 9px;border-radius:999px}
.badge.open{background:color-mix(in srgb,var(--label2) 16%,transparent);color:var(--label2)}
.badge.closed{background:color-mix(in srgb,var(--on) 18%,transparent);color:var(--on)}
.bars{display:inline-flex;gap:2px;align-items:flex-end;height:14px}
.bars i{width:3px;background:var(--label3);border-radius:1px}
.bars i.on{background:var(--accent)}

/* Banner + toast + login */
.banner{background:color-mix(in srgb,#ff9f0a 16%,var(--group));border:.5px solid #ff9f0a;border-radius:14px;
  padding:12px 15px;font-size:13.5px;margin-bottom:14px;display:flex;gap:10px;align-items:center}
.toast{position:fixed;left:50%;bottom:calc(24px + env(safe-area-inset-bottom));transform:translate(-50%,20px);
  background:rgba(30,30,32,.92);color:#fff;backdrop-filter:blur(20px);padding:12px 20px;border-radius:14px;
  font-size:14.5px;font-weight:500;box-shadow:0 12px 40px rgba(0,0,0,.35);opacity:0;pointer-events:none;
  transition:opacity .3s var(--spring),transform .3s var(--spring);z-index:50}
.toast.show{opacity:1;transform:translate(-50%,0)}
.overlay{position:fixed;inset:0;z-index:60;display:grid;place-items:center;padding:24px;background:var(--bg)}
.login{width:100%;max-width:340px;text-align:center}
.login .brand{width:56px;height:56px;border-radius:15px;font-size:26px;margin:0 auto 16px}
.login h1{font-size:24px;font-weight:700;letter-spacing:-.03em}
.login p{color:var(--label2);font-size:14px;margin:6px 0 22px}
.login .group{margin-bottom:16px;text-align:left}
.login .row{padding:0}
.login .row+.row::before{left:16px}
.err{color:var(--danger);font-size:13.5px;min-height:18px;margin-top:2px}
a.row{color:inherit;text-decoration:none}
a.row .v{color:var(--accent);font-weight:600}
@media (prefers-reduced-motion:reduce){*{animation:none!important;transition:none!important}}
</style>
</head>
<body>

<div id="login" class="overlay">
  <form class="login" onsubmit="doLogin(event)">
    <div class="brand">H</div>
    <h1>HDC Node</h1><p>Sign in to manage this node</p>
    <div class="group">
      <label class="fld"><input id="lu" type="text" placeholder="Username" autocomplete="username" value="admin"></label>
      <div class="row" style="padding:0"><label class="fld" style="width:100%"><input id="lp" type="password" placeholder="Password" autocomplete="current-password"></label></div>
    </div>
    <button class="btn">Sign In</button>
    <div id="lerr" class="err"></div>
  </form>
</div>

<div id="setup" class="overlay hidden">
  <form class="login" onsubmit="doSetup(event)">
    <div class="brand">H</div>
    <h1>Secure this node</h1><p>Create your administrator account to continue</p>
    <div class="group" style="text-align:left">
      <label class="fld"><span class="cap">Current password</span><input id="suCur" type="password" autocomplete="current-password"></label>
      <label class="fld"><span class="cap">New username</span><input id="suUser" type="text" value="admin" autocapitalize="off" autocorrect="off" spellcheck="false" autocomplete="username"></label>
      <label class="fld"><span class="cap">New password</span><input id="suNew" type="password" autocomplete="new-password" placeholder="at least 8 characters"></label>
      <label class="fld"><span class="cap">Confirm password</span><input id="suNew2" type="password" autocomplete="new-password"></label>
    </div>
    <button class="btn">Create Account</button>
    <div id="suErr" class="err"></div>
  </form>
</div>

<div id="app" class="hidden">
<header>
  <div class="brand">H</div>
  <div class="hgroup"><div class="n" id="hName">HDC Node</div><div class="l" id="hLoc">—</div></div>
  <div class="grow"></div>
  <span class="chip"><span class="dot" id="netDot"></span><span id="netTxt">—</span></span>
  <button class="txtbtn" onclick="logout()">Sign Out</button>
</header>

<div class="wrap">
  <nav id="nav">
    <button class="active" data-tab="dash" onclick="tab('dash')"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><path d="M4 20a8 8 0 1 1 16 0"/><path d="M12 14l4.5-4.5"/></svg><span>Dashboard</span></button>
    <button data-tab="net" onclick="tab('net')"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="9"/><path d="M3 12h18"/><path d="M12 3a15 15 0 0 1 0 18 15 15 0 0 1 0-18"/></svg><span>Network</span></button>
    <button data-tab="snmp" onclick="tab('snmp')"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="1.8"/><path d="M8.5 8.5a5 5 0 0 0 0 7"/><path d="M15.5 8.5a5 5 0 0 1 0 7"/><path d="M6 6a9 9 0 0 0 0 12"/><path d="M18 6a9 9 0 0 1 0 12"/></svg><span>SNMP</span></button>
    <button data-tab="relay" onclick="tab('relay')"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><path d="M13 2 4 14h7l-1 8 9-12h-7l1-8z"/></svg><span>Relays</span></button>
    <button data-tab="sec" onclick="tab('sec')"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><rect x="5" y="11" width="14" height="10" rx="2.2"/><path d="M8 11V7a4 4 0 0 1 8 0v4"/></svg><span>Security</span></button>
    <button data-tab="sys" onclick="tab('sys')"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round"><path d="M4 6h9M17 6h3M4 12h3M11 12h9M4 18h7M15 18h5"/><circle cx="15" cy="6" r="2"/><circle cx="9" cy="12" r="2"/><circle cx="13" cy="18" r="2"/></svg><span>System</span></button>
  </nav>

  <main>
    <div id="pwBanner" class="banner hidden"><svg viewBox="0 0 24 24" width="18" height="18" fill="none" stroke="#ff9f0a" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" style="flex:0 0 auto"><path d="M12 3 2 20h20L12 3z"/><path d="M12 10v4"/><path d="M12 17.5v.01"/></svg><span>Default password in use — set a new one under <b>Security</b>.</span></div>

    <section data-pane="dash">
      <div class="hero">
        <div>
          <div class="big mono"><span id="mTc">—</span><span class="deg">°C</span></div>
          <div class="cap"><span id="mTf">—</span> °F · updated live</div>
        </div>
        <div class="side">
          <div class="s"><div class="l">Humidity</div><div class="val mono"><span id="mH">—</span>%</div></div>
          <div class="s"><div class="l">Airflow</div><div class="val mono" id="mA">—</div></div>
        </div>
      </div>
      <div class="tiles">
        <div class="tile"><div class="l">Contact 1</div><div class="val"><span class="badge open" id="mC1">—</span></div></div>
        <div class="tile"><div class="l">Contact 2</div><div class="val"><span class="badge open" id="mC2">—</span></div></div>
        <div class="tile"><div class="l">Uptime</div><div class="val mono" id="mUp" style="font-size:19px">—</div></div>
        <div class="tile"><div class="l">Address</div><div class="val mono" id="mIp" style="font-size:17px">—</div></div>
      </div>

      <div class="gtitle">Relays</div>
      <div class="group">
        <div class="row"><div><div class="k" id="r1lbl">Relay 1</div><div class="sub">Applies instantly · persists</div></div>
          <label class="sw"><input type="checkbox" id="r1" onchange="setRelay(1,this.checked)"><span class="tk"></span></label></div>
        <div class="row"><div><div class="k" id="r2lbl">Relay 2</div><div class="sub">Applies instantly · persists</div></div>
          <label class="sw"><input type="checkbox" id="r2" onchange="setRelay(2,this.checked)"><span class="tk"></span></label></div>
      </div>
    </section>

    <section data-pane="net" class="hidden">
      <div class="gtitle">Connection</div>
      <div class="group">
        <div class="row"><div class="k">Active interface</div><div class="v" id="nMode">—</div></div>
        <div class="row"><div class="k">Ethernet</div><div class="v" id="nEth">—</div></div>
        <div class="row"><div class="k">Wi-Fi</div><div class="v"><span class="bars" id="nBars"></span><span id="nWifi">—</span></div></div>
      </div>

      <div class="gtitle">Device hostname</div>
      <div class="group">
        <label class="fld"><input id="hostname" type="text" placeholder="hdc-node-001" autocapitalize="off" autocorrect="off" spellcheck="false"></label>
      </div>
      <p class="gnote">Announced to DHCP so the node appears by name in the router's lease list (letters, digits and hyphens only).</p>

      <div class="gtitle">Ethernet</div>
      <div class="group">
        <div class="row"><div class="k">Use DHCP</div><label class="sw"><input type="checkbox" id="dhcp"><span class="tk"></span></label></div>
        <label class="fld"><span class="cap">Static IP (fallback)</span><input id="ip" type="text" placeholder="192.168.9.200"></label>
        <label class="fld"><span class="cap">Subnet mask</span><input id="mask" type="text" placeholder="255.255.255.0"></label>
        <label class="fld"><span class="cap">Gateway</span><input id="gw" type="text"></label>
        <label class="fld"><span class="cap">DNS</span><input id="dns" type="text"></label>
      </div>
      <div class="gtitle">Wi-Fi</div>
      <div class="group">
        <div class="row"><div class="k">Enable Wi-Fi</div><label class="sw"><input type="checkbox" id="wifiEn"><span class="tk"></span></label></div>
        <label class="fld"><span class="cap">Network name (SSID)</span><input id="wifiSsid" type="text"></label>
        <label class="fld"><span class="cap">Password</span><input id="wifiPass" type="password" placeholder="•••••••• (unchanged)" autocomplete="off"></label>
      </div>
      <p class="gnote">DHCP is attempted first; the static settings are the fallback. Ethernet and Wi-Fi can run together — the node is reachable on whichever connects. Changes apply after reboot.</p>
      <div class="saverow"><button class="btn" onclick="saveSettings()">Save & Apply</button></div>
    </section>

    <section data-pane="snmp" class="hidden">
      <div class="gtitle">SNMP v2c</div>
      <div class="group">
        <div class="row"><div class="k">Enable agent</div><label class="sw"><input type="checkbox" id="snmpEn"><span class="tk"></span></label></div>
        <label class="fld"><span class="cap">Read community</span><input id="snmpRo" type="text"></label>
        <div class="row"><div class="k">Allow relay writes</div><label class="sw"><input type="checkbox" id="snmpWrEn"><span class="tk"></span></label></div>
        <label class="fld"><span class="cap">Write community</span><input id="snmpRw" type="text"></label>
      </div>
      <p class="gnote">Monitoring is read-only via the read community. SNMPv2c has no encryption — keep the agent on a trusted management VLAN and prefer this HTTPS API for secure control.</p>
      <div class="saverow"><button class="btn" onclick="saveSettings()">Save</button></div>
    </section>

    <section data-pane="relay" class="hidden">
      <div class="gtitle">Relay names</div>
      <div class="group">
        <label class="fld"><span class="cap">Relay 1</span><input id="r1name" type="text"></label>
        <label class="fld"><span class="cap">Relay 2</span><input id="r2name" type="text"></label>
      </div>
      <div class="saverow"><button class="btn" onclick="saveSettings()">Save</button></div>
    </section>

    <section data-pane="sec" class="hidden">
      <div class="gtitle">Administrator</div>
      <div class="group">
        <label class="fld"><span class="cap">Username</span><input id="admUser" type="text" autocomplete="username"></label>
        <label class="fld"><span class="cap">Current password</span><input id="pwCur" type="password" autocomplete="current-password"></label>
        <label class="fld"><span class="cap">New password</span><input id="pwNew" type="password" autocomplete="new-password" placeholder="at least 6 characters"></label>
      </div>
      <p class="gnote">These credentials protect the web UI and the REST API.</p>
      <div class="saverow"><button class="btn" onclick="changePw()">Update Credentials</button></div>
    </section>

    <section data-pane="sys" class="hidden">
      <div class="gtitle">Identity</div>
      <div class="group">
        <label class="fld"><span class="cap">Node name</span><input id="node" type="text"></label>
        <label class="fld"><span class="cap">Location</span><input id="loc" type="text"></label>
        <label class="fld"><span class="cap">Contact</span><input id="contact" type="text"></label>
        <label class="fld"><span class="cap">Sensor interval (ms)</span><input id="interval" type="number" min="250" max="60000" step="250"></label>
      </div>
      <div class="saverow"><button class="btn" onclick="saveSettings()">Save</button></div>

      <div class="gtitle">Prometheus &amp; Grafana</div>
      <div class="group">
        <div class="row"><div class="k">Expose /metrics</div><label class="sw"><input type="checkbox" id="metricsEn"><span class="tk"></span></label></div>
        <label class="fld"><span class="cap">Scrape bearer token (blank = open on trusted VLAN)</span>
          <input id="metricsTok" type="text" autocapitalize="off" autocorrect="off" spellcheck="false" placeholder="optional">
          <div class="hint">Endpoint: <code>https://<span id="metricsUrl">this-node</span>/metrics</code></div></label>
      </div>
      <div class="saverow"><button class="btn" onclick="saveSettings()">Save metrics</button></div>

      <div class="gtitle">Downloads</div>
      <div class="group">
        <a class="row tap" href="/api/mib" download><div><div class="k">SNMP MIB</div><div class="sub">HDC-SENSE-CONTROL-MIB.txt</div></div><div class="v">Download</div></a>
        <a class="row tap" href="/api/zabbix" download><div><div class="k">Zabbix 7.0 template</div><div class="sub">YAML · read community pre-filled</div></div><div class="v">Download</div></a>
        <a class="row tap" href="/api/grafana" download><div><div class="k">Grafana dashboard</div><div class="sub">JSON · Prometheus datasource</div></div><div class="v">Download</div></a>
      </div>
      <p class="gnote">Zabbix: Data collection → Templates → Import (community <code>{$SNMP_COMMUNITY}</code>). Grafana: Dashboards → Import. Point Prometheus at <code>/metrics</code>.</p>

      <div class="gtitle">Maintenance</div>
      <div class="group">
        <div class="row"><div class="k">Firmware</div><div class="v" id="fw">—</div></div>
        <div class="row"><div class="k">MAC</div><div class="v mono" id="mac">—</div></div>
      </div>
      <div class="saverow" style="display:grid;gap:10px">
        <button class="btn tinted" onclick="reboot()">Reboot Node</button>
        <button class="btn danger" onclick="factory()">Factory Reset…</button>
      </div>
      <p class="gnote">Factory reset erases all settings and the TLS certificate, then reboots. You can also hold the BOOT button for 5&nbsp;s at power-up.</p>
    </section>
  </main>
</div>
</div>

<div id="toast" class="toast"></div>

<script>
let S={},curPass='';
function $(i){return document.getElementById(i)}
function toast(m){let t=$('toast');t.textContent=m;t.classList.add('show');clearTimeout(t._t);t._t=setTimeout(()=>t.classList.remove('show'),2400)}
async function api(p,m,b){let o={method:m||'GET',headers:{},credentials:'same-origin'};
  if(b!==undefined){o.headers['Content-Type']='application/json';o.body=JSON.stringify(b)}
  let r=await fetch(p,o);if(r.status===401){showLogin();throw new Error('unauth')}
  if(!r.ok){let t='';try{t=(await r.json()).error}catch(e){}throw new Error(t||('HTTP '+r.status))}
  let c=r.headers.get('content-type')||'';return c.includes('json')?r.json():r.text()}
function hideAll(){['login','setup','app'].forEach(i=>$(i).classList.add('hidden'))}
function showLogin(){hideAll();$('login').classList.remove('hidden')}
function showApp(){hideAll();$('app').classList.remove('hidden')}
function showSetup(){hideAll();$('setup').classList.remove('hidden');if(curPass)suCur.value=curPass}
async function doLogin(e){e.preventDefault();try{let r=await api('/api/login','POST',{username:lu.value,password:lp.value});
  curPass=lp.value;if(r.mustChangePassword){showSetup()}else{showApp();await loadAll()}}
  catch(err){$('lerr').textContent='Incorrect username or password'}}
async function doSetup(e){e.preventDefault();$('suErr').textContent='';
  if(suNew.value.length<8){$('suErr').textContent='Password must be at least 8 characters';return}
  if(suNew.value!==suNew2.value){$('suErr').textContent='Passwords do not match';return}
  try{await api('/api/password','POST',{username:suUser.value,current:suCur.value,new:suNew.value});
    curPass='';showApp();await loadAll();toast('Administrator account secured')}
  catch(err){$('suErr').textContent=err.message||'Setup failed'}}
async function logout(){try{await api('/api/logout','POST',{})}catch(e){}showLogin()}
function tab(id){document.querySelectorAll('#nav button').forEach(b=>b.classList.toggle('active',b.dataset.tab===id));
  document.querySelectorAll('[data-pane]').forEach(p=>{let on=p.dataset.pane===id;p.classList.toggle('hidden',!on);
    if(on){p.style.animation='none';p.offsetHeight;p.style.animation=''}})}
function bars(rssi){let n=rssi>=-55?4:rssi>=-67?3:rssi>=-78?2:rssi>=-88?1:0,h=[6,9,12,14],o='';
  for(let i=0;i<4;i++)o+='<i class="'+(i<n?'on':'')+'" style="height:'+h[i]+'px"></i>';return o}
async function loadSettings(){
  S=await api('/api/settings');
  hName.textContent=S.nodeName;hLoc.textContent=S.location;fw.textContent='v'+S.firmware;
  node.value=S.nodeName;loc.value=S.location;contact.value=S.contact;interval.value=S.sensorIntervalMs;hostname.value=S.hostname;
  dhcp.checked=S.dhcpEnabled;ip.value=S.staticIp;mask.value=S.staticMask;gw.value=S.staticGw;dns.value=S.staticDns;
  wifiEn.checked=S.wifiEnabled;wifiSsid.value=S.wifiSsid;wifiPass.value='';
  wifiPass.placeholder=S.wifiConfigured?'•••••••• (unchanged)':'password';
  snmpEn.checked=S.snmpEnabled;snmpWrEn.checked=S.snmpWriteEnabled;snmpRo.value=S.snmpReadCommunity;snmpRw.value=S.snmpWriteCommunity;
  r1name.value=S.relay1Name;r2name.value=S.relay2Name;r1lbl.textContent=S.relay1Name;r2lbl.textContent=S.relay2Name;
  admUser.value=S.adminUser;$('pwBanner').classList.toggle('hidden',!S.mustChangePassword);
  metricsEn.checked=S.metricsEnabled;metricsTok.value=S.metricsToken||'';$('metricsUrl').textContent=location.host}
async function loadStatus(){try{let d=await api('/api/status');
  mTc.textContent=d.tempC.toFixed(1);mTf.textContent=d.tempF.toFixed(1);mH.textContent=d.humidity.toFixed(0);mA.textContent=d.airflow;
  setBadge('mC1',d.contact1);setBadge('mC2',d.contact2);mUp.textContent=d.uptime;mIp.textContent=d.ip;mac.textContent=d.mac||S.mac||'—';
  r1.checked=d.relay1;r2.checked=d.relay2;
  netDot.className='dot '+(d.link?'on':(d.apActive?'off':'off'));netTxt.textContent=d.mode+(d.ip&&d.ip!=='0.0.0.0'?' · '+d.ip:'');
  nMode.textContent=d.mode;nEth.textContent=d.ethLink?(d.ethIp+' · link up'):'no link';
  nWifi.textContent=d.wifiUp?(d.wifiIp+' · '+d.wifiRssi+' dBm'):(d.apActive?('AP '+d.ssid):'not connected');
  nBars.innerHTML=d.wifiUp?bars(d.wifiRssi):''}catch(e){}}
function setBadge(id,closed){let e=$(id);e.textContent=closed?'Closed':'Open';e.className='badge '+(closed?'closed':'open')}
async function saveSettings(){let b={nodeName:node.value,location:loc.value,contact:contact.value,hostname:hostname.value,sensorIntervalMs:+interval.value,
  dhcpEnabled:dhcp.checked,staticIp:ip.value,staticMask:mask.value,staticGw:gw.value,staticDns:dns.value,
  wifiEnabled:wifiEn.checked,wifiSsid:wifiSsid.value,wifiPass:wifiPass.value,
  snmpEnabled:snmpEn.checked,snmpWriteEnabled:snmpWrEn.checked,snmpReadCommunity:snmpRo.value,snmpWriteCommunity:snmpRw.value,
  relay1Name:r1name.value,relay2Name:r2name.value,metricsEnabled:metricsEn.checked,metricsToken:metricsTok.value};
  try{await api('/api/settings','POST',b);toast('Saved · network changes need a reboot');await loadSettings()}catch(e){toast('Save failed: '+e.message)}}
async function setRelay(n,on){try{await api('/api/relay','POST',{relay:n,state:on?1:0})}catch(e){toast('Relay change failed')}}
async function changePw(){try{await api('/api/password','POST',{username:admUser.value,current:pwCur.value,new:pwNew.value});
  pwCur.value='';pwNew.value='';toast('Credentials updated');await loadSettings()}catch(e){toast(e.message)}}
async function reboot(){if(!confirm('Reboot the node now?'))return;try{await api('/api/reboot','POST',{})}catch(e){}toast('Rebooting…')}
async function factory(){if(!confirm('Factory reset erases ALL settings and the TLS certificate, then reboots. Continue?'))return;
  try{await api('/api/factory-reset','POST',{})}catch(e){}toast('Restoring factory defaults…')}
async function loadAll(){await loadSettings();await loadStatus()}
async function init(){try{let d=await api('/api/status');if(d.mustChangePassword){showSetup()}else{showApp();await loadAll()}}catch(e){showLogin()}
  setInterval(()=>{if(!$('app').classList.contains('hidden'))loadStatus()},2000)}
init();
</script>
</body></html>)HTMLDOC";
