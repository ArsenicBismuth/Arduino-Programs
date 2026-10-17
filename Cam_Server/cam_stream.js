// Camera panel for the ESPHome web UI (loaded via web_server: js_include).
//
// The MJPEG stream lives on its own server (esp32_camera_web_server, port 8080),
// so the control UI on port 80 never blocks while the feed is watched.
//
// The stream connection is opened only when the user presses "Start" and torn
// down again on "Stop" - the page makes no camera request on load.

const STREAM_PORT = 8080;
const streamUrl = `${location.protocol}//${location.hostname}:${STREAM_PORT}/`;

const SYMBOL_EXPAND = "\u2922"; // ⤢  bigger window (not fullscreen)
const SYMBOL_FULLSCREEN = "\u26F6"; // ⛶  fullscreen (same symbol to enter/exit)

// Tiny inline SVG shown while idle, so the panel has a stable footprint.
const idleImage =
  "data:image/svg+xml;utf8," +
  encodeURIComponent(
    '<svg xmlns="http://www.w3.org/2000/svg" width="480" height="270">' +
      '<rect width="100%" height="100%" fill="#2a2a2c"/>' +
      '<text x="50%" y="50%" fill="#888" font-family="system-ui" font-size="16" ' +
      'text-anchor="middle" dominant-baseline="middle">Camera idle</text></svg>'
  );

const panel = document.createElement("div");
panel.id = "cam-stream-panel";

const header = document.createElement("div");
header.id = "cam-stream-header";

const title = document.createElement("span");
title.textContent = "Camera";

const actions = document.createElement("div");
actions.id = "cam-stream-actions";

const button = document.createElement("button");
button.id = "cam-stream-toggle";
button.type = "button";
button.textContent = "Start";

const expandButton = document.createElement("button");
expandButton.id = "cam-stream-expand";
expandButton.type = "button";
expandButton.textContent = SYMBOL_EXPAND;
expandButton.title = "Bigger window";

const fsButton = document.createElement("button");
fsButton.id = "cam-stream-fullscreen";
fsButton.type = "button";
fsButton.textContent = SYMBOL_FULLSCREEN;
fsButton.title = "Fullscreen";

const img = document.createElement("img");
img.id = "cam-stream-img";
img.alt = "Camera stream";
img.src = idleImage;

let streaming = false;
button.addEventListener("click", () => {
  if (streaming) {
    // Stop: replacing the src closes the MJPEG connection to :8080.
    img.src = idleImage;
    panel.classList.remove("streaming");
    button.textContent = "Start";
  } else {
    // Start: assigning the src opens the MJPEG connection to :8080.
    img.src = streamUrl;
    panel.classList.add("streaming");
    button.textContent = "Stop";
  }
  streaming = !streaming;
});

expandButton.addEventListener("click", () => {
  const expanded = panel.classList.toggle("expanded");
  expandButton.title = expanded ? "Normal window" : "Bigger window";
});

fsButton.addEventListener("click", () => {
  if (document.fullscreenElement) {
    document.exitFullscreen();
  } else if (panel.requestFullscreen) {
    panel.requestFullscreen();
  }
});

// Same ⛶ symbol in both directions; only the tooltip changes.
document.addEventListener("fullscreenchange", () => {
  fsButton.title =
    document.fullscreenElement === panel ? "Exit fullscreen" : "Fullscreen";
});

actions.append(button, expandButton, fsButton);
header.append(title, actions);
panel.append(header, img);
document.body.appendChild(panel);

// --- Help button: floating "?" that reveals the device's HTTP endpoints -------
// Everything here shares the page's host; only the MJPEG / snapshot servers
// use their own ports (see web_server + esp32_camera_web_server in cam.yaml).
const SYMBOL_HELP = "?";

const helpHost = `${location.protocol}//${location.hostname}`;
const HELP_LINKS = [
  ["/file", "Browse and download the SD card", `${helpHost}/file`],
  ["/latest", "Newest snapshot as an image", `${helpHost}/latest`],
  [":8080", "Live MJPEG stream", `${helpHost}:8080`],
  [":8081", "Single snapshot", `${helpHost}:8081`],
];

const help = document.createElement("div");
help.id = "cam-help";

const helpButton = document.createElement("button");
helpButton.id = "cam-help-toggle";
helpButton.type = "button";
helpButton.textContent = SYMBOL_HELP;
helpButton.title = "Available URLs";
helpButton.setAttribute("aria-label", "Available URLs");
helpButton.setAttribute("aria-expanded", "false");
helpButton.setAttribute("aria-controls", "cam-help-panel");

const helpPanel = document.createElement("div");
helpPanel.id = "cam-help-panel";

const helpTitle = document.createElement("div");
helpTitle.id = "cam-help-title";
helpTitle.textContent = "Available URLs";

const helpList = document.createElement("ul");
helpList.id = "cam-help-list";
for (const [label, description, href] of HELP_LINKS) {
  const item = document.createElement("li");
  const link = document.createElement("a");
  link.href = href;
  link.textContent = label;
  link.title = `${description} - ${href}`;
  link.target = "_blank";
  link.rel = "noopener";
  item.appendChild(link);
  helpList.appendChild(item);
}

function setHelpOpen(open) {
  help.classList.toggle("open", open);
  helpButton.setAttribute("aria-expanded", open ? "true" : "false");
}

helpButton.addEventListener("click", () => setHelpOpen(!help.classList.contains("open")));

// Dismiss when clicking anywhere else or pressing Escape.
document.addEventListener("click", (event) => {
  if (help.classList.contains("open") && !help.contains(event.target))
    setHelpOpen(false);
});
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape")
    setHelpOpen(false);
});

helpPanel.append(helpTitle, helpList);
help.append(helpButton, helpPanel);
document.body.appendChild(help);
