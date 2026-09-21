// The growth report card: live numbers from the optimizer, so the animation
// explains itself. Everything shown is real data from the run — nothing is
// decorative.
//
// "Sag under load" is the compliance. With a single unit load, compliance
// (f . u) is literally the vertical displacement of the loaded point, so
// "100% -> 14%" means the bridge now sags to 14% of what it did when the
// stone was first spread evenly. Total stone never changes during a run: the
// optimizer only moves it.
import { HEAT_STOPS } from "./render.js";

const card = document.getElementById("report");
const iterEl = document.getElementById("rep-iter");
const stoneEl = document.getElementById("rep-stone");
const sagEl = document.getElementById("rep-sag");
const noteEl = document.getElementById("rep-note");
const spark = document.getElementById("rep-spark");
const legendBar = document.getElementById("rep-legend");

legendBar.style.background = `linear-gradient(90deg, ${HEAT_STOPS.map(
  ([t, c]) => `rgb(${c[0]},${c[1]},${c[2]}) ${t * 100}%`
).join(", ")})`;

export function showReport() {
  card.classList.remove("hidden");
}

export function hideReport() {
  card.classList.add("hidden");
}

// What the optimizer is doing right now, in words, from how much it is
// still moving stone around.
function narrate({ iteration, maxChange, converged, finished }) {
  if (converged) return "Settled: no stone left worth moving.";
  if (finished) return "Stopped at the iteration limit. The shape has stopped changing.";
  if (iteration <= 2) return "Stone spread evenly. Measuring where it strains.";
  if (maxChange > 0.1) return "Moving stone from idle areas to where it carries load.";
  return "Fine-tuning the edges.";
}

function drawSparkline(history) {
  const ctx = spark.getContext("2d");
  const w = spark.width;
  const h = spark.height;
  ctx.clearRect(0, 0, w, h);
  if (history.length < 2) return;

  const start = history[0];
  const pad = 8;
  const x = (i) => pad + (i / Math.max(1, history.length - 1)) * (w - 2 * pad);
  const y = (c) => pad + (1 - Math.min(1, c / start)) * (h - 2 * pad);

  ctx.strokeStyle = "rgba(255,255,255,0.10)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(pad, y(start));
  ctx.lineTo(w - pad, y(start));
  ctx.stroke();

  const fill = ctx.createLinearGradient(0, 0, 0, h);
  fill.addColorStop(0, "rgba(230,192,123,0.35)");
  fill.addColorStop(1, "rgba(230,192,123,0)");
  ctx.beginPath();
  ctx.moveTo(x(0), h - pad);
  history.forEach((c, i) => ctx.lineTo(x(i), y(c)));
  ctx.lineTo(x(history.length - 1), h - pad);
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.fill();

  ctx.beginPath();
  history.forEach((c, i) => (i === 0 ? ctx.moveTo(x(i), y(c)) : ctx.lineTo(x(i), y(c))));
  ctx.strokeStyle = "#e6c07b";
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.stroke();

  const last = history.length - 1;
  ctx.beginPath();
  ctx.arc(x(last), y(history[last]), 5, 0, Math.PI * 2);
  ctx.fillStyle = "#fff4dc";
  ctx.fill();
}

export function updateReport({ iteration, history, maxChange, converged, finished, materialKg }) {
  iterEl.textContent = `iteration ${iteration}`;
  stoneEl.textContent = `${materialKg} kg, unchanged`;
  const ratio = history[history.length - 1] / history[0];
  sagEl.textContent = `100% → ${Math.round(ratio * 100)}%`;
  noteEl.textContent = narrate({ iteration, maxChange, converged, finished });
  drawSparkline(history);
}
