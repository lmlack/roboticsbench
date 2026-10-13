// Tab routing, theme, and lazy start of each tool. A tool's module loads the
// first time its tab opens, so the PID page never pays for three.js. The home
// page is static and has no module.

const TOOLS = {
  learn: () => import("./learn.js"),
  rotations: () => import("./rotations.js"),
  urdf: () => import("./urdf.js"),
  pid: () => import("./pid.js"),
  kalman: () => import("./kalman.js"),
  localization: () => import("./localization.js"),
  planning: () => import("./planning.js"),
};
const started = new Map();

async function show(name) {
  if (!(name in TOOLS)) name = "home";
  for (const el of document.querySelectorAll("[data-tool]")) el.hidden = el.dataset.tool !== name;
  for (const a of document.querySelectorAll("[data-tab]")) {
    if (a.dataset.tab === name) a.setAttribute("aria-current", "page");
    else a.removeAttribute("aria-current");
  }
  for (const [key, tool] of started) tool.setActive?.(key === name);
  if (name === "home") return null;
  if (!started.has(name)) {
    const section = document.querySelector(`[data-tool="${name}"]`);
    try {
      const mod = await TOOLS[name]();
      started.set(name, mod.start(section));
    } catch (err) {
      console.error(err);
      section.insertAdjacentHTML("beforeend", '<p class="note" style="text-align:center">This tool failed to load. Try reloading the page.</p>');
      started.set(name, {});
    }
    started.get(name).setActive?.(true);
  }
  return started.get(name);
}

function route() {
  show(location.hash.slice(1));
}
window.addEventListener("hashchange", route);
route();

// Other tools hand a rotation to the converter through this event.
window.addEventListener("rb:open-rotation", async (e) => {
  location.hash = "#rotations";
  const tool = await show("rotations");
  tool.setQuaternion(e.detail.q);
  window.scrollTo({ top: 0, behavior: "smooth" });
});

// ---- Theme ----
function applyTheme(dark) {
  if (dark) document.documentElement.dataset.theme = "dark";
  else delete document.documentElement.dataset.theme;
  try {
    if (dark) localStorage.setItem("rb-theme", "dark");
    else localStorage.removeItem("rb-theme");
  } catch {
    // Private browsing: it applies, it just is not remembered.
  }
  syncThemeButtons();
  window.dispatchEvent(new Event("rb:theme"));
}
function syncThemeButtons() {
  const dark = document.documentElement.dataset.theme === "dark";
  for (const b of document.querySelectorAll("[data-theme-set]")) {
    b.setAttribute("aria-pressed", String((b.dataset.themeSet === "dark") === dark));
  }
}
for (const b of document.querySelectorAll("[data-theme-set]")) {
  b.addEventListener("click", () => applyTheme(b.dataset.themeSet === "dark"));
}
syncThemeButtons();
