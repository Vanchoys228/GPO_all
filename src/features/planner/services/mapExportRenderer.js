import { HALF_HEIGHT, HALF_WIDTH } from "../../../lib/zonePlanner";

const clampCanvasValue = (value, min, max) => Math.max(min, Math.min(max, value));

const loadCanvasImage = (src) =>
  new Promise((resolve) => {
    if (!src) {
      resolve(null);
      return;
    }

    const image = new Image();
    image.onload = () => resolve(image);
    image.onerror = () => resolve(null);
    image.src = src;
  });

const drawCoveredImage = (ctx, image, x, y, width, height) => {
  const scale = Math.max(width / image.width, height / image.height);
  const sourceWidth = width / scale;
  const sourceHeight = height / scale;
  const sourceX = Math.max(0, (image.width - sourceWidth) / 2);
  const sourceY = Math.max(0, (image.height - sourceHeight) / 2);
  ctx.drawImage(image, sourceX, sourceY, sourceWidth, sourceHeight, x, y, width, height);
};
export const drawCameraMapExport = async (ctx, exportCanvas, selectedMap, camera) => {
  const cells = Array.isArray(selectedMap?.cells) ? selectedMap.cells : [];
  const freeCells = Array.isArray(selectedMap?.freeCells) ? selectedMap.freeCells : [];
  const width = exportCanvas.width;
  const height = exportCanvas.height;
  const centerX = width / 2;
  const floorTop = height * 0.14;
  const floorBottom = height * 0.94;
  const farWidth = width * 0.40;
  const nearWidth = width * 0.96;
  const cameraFrame = await loadCanvasImage(camera?.frameDataUrl);

  const background = ctx.createLinearGradient(0, 0, width, height);
  background.addColorStop(0, "#07111f");
  background.addColorStop(0.48, "#121827");
  background.addColorStop(1, "#25113a");
  ctx.fillStyle = background;
  ctx.fillRect(0, 0, width, height);

  ctx.save();
  ctx.beginPath();
  ctx.moveTo(centerX - farWidth / 2, floorTop);
  ctx.lineTo(centerX + farWidth / 2, floorTop);
  ctx.lineTo(centerX + nearWidth / 2, floorBottom);
  ctx.lineTo(centerX - nearWidth / 2, floorBottom);
  ctx.closePath();
  ctx.clip();

  const floorGradient = ctx.createLinearGradient(0, floorTop, 0, floorBottom);
  floorGradient.addColorStop(0, "rgba(76,29,149,0.34)");
  floorGradient.addColorStop(0.58, "rgba(30,41,59,0.78)");
  floorGradient.addColorStop(1, "rgba(2,6,23,0.94)");
  ctx.fillStyle = floorGradient;
  ctx.fillRect(0, floorTop, width, floorBottom - floorTop);

  if (cameraFrame) {
    ctx.save();
    ctx.globalAlpha = 0.18;
    ctx.filter = "saturate(1.15) contrast(1.15)";
    drawCoveredImage(ctx, cameraFrame, centerX - nearWidth / 2, floorTop, nearWidth, floorBottom - floorTop);
    ctx.restore();
  }

  for (let depthIndex = 0; depthIndex <= 18; depthIndex += 1) {
    const t = depthIndex / 18;
    const y = floorTop + (floorBottom - floorTop) * t;
    const lineWidth = farWidth + (nearWidth - farWidth) * t;
    const alpha = 0.08 + t * 0.10;
    ctx.strokeStyle = `rgba(148,163,184,${alpha})`;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(centerX - lineWidth / 2, y);
    ctx.lineTo(centerX + lineWidth / 2, y);
    ctx.stroke();
  }
  for (let lateralIndex = -10; lateralIndex <= 10; lateralIndex += 1) {
    const offset = lateralIndex / 20;
    ctx.strokeStyle = "rgba(148,163,184,0.12)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(centerX + offset * farWidth, floorTop);
    ctx.lineTo(centerX + offset * nearWidth, floorBottom);
    ctx.stroke();
  }

  const project = (cell) => {
    const x = clampCanvasValue(Number(cell.x), -HALF_WIDTH, HALF_WIDTH);
    const y = clampCanvasValue(Number(cell.y), -HALF_HEIGHT, HALF_HEIGHT);
    const depth = clampCanvasValue((HALF_HEIGHT - y) / (HALF_HEIGHT * 2), 0, 1);
    const lateral = x / (HALF_WIDTH * 2);
    const widthAtDepth = farWidth + (nearWidth - farWidth) * depth;
    return {
      x: centerX + lateral * widthAtDepth,
      y: floorTop + depth * (floorBottom - floorTop),
      depth,
    };
  };

  const orderedCells = cells
    .map((cell) => ({ cell, point: project(cell) }))
    .sort((left, right) => left.point.depth - right.point.depth);
  const orderedFreeCells = freeCells
    .map((cell) => ({ cell, point: project(cell) }))
    .sort((left, right) => left.point.depth - right.point.depth);

  orderedFreeCells.forEach(({ cell, point }) => {
    const confidence = Math.max(0, Number(cell.confidence) || 0);
    const strength = clampCanvasValue(confidence / 12, 0.12, 1);
    const radius = 3 + point.depth * 9 + strength * 4;
    const alpha = clampCanvasValue(0.10 + strength * 0.28, 0.12, 0.42);

    ctx.fillStyle = `rgba(45,212,191,${alpha})`;
    ctx.strokeStyle = `rgba(103,232,249,${alpha * 0.70})`;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.ellipse(point.x, point.y, radius * 1.75, radius * 0.62, 0, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  });

  orderedCells.forEach(({ cell, point }) => {
    const confidence = Math.max(0, Number(cell.confidence) || 0);
    const strength = clampCanvasValue(confidence / 9, 0.24, 1);
    const radius = 4 + point.depth * 8 + strength * 4;
    const columnHeight = 16 + point.depth * 48 + strength * 34;
    const alpha = clampCanvasValue(0.28 + strength * 0.64, 0.32, 0.96);

    const shadow = ctx.createRadialGradient(point.x, point.y, 0, point.x, point.y, radius * 3.2);
    shadow.addColorStop(0, `rgba(217,70,239,${alpha * 0.46})`);
    shadow.addColorStop(0.45, `rgba(124,58,237,${alpha * 0.24})`);
    shadow.addColorStop(1, "rgba(124,58,237,0)");
    ctx.fillStyle = shadow;
    ctx.beginPath();
    ctx.ellipse(point.x, point.y, radius * 2.6, radius * 0.95, 0, 0, Math.PI * 2);
    ctx.fill();

    const column = ctx.createLinearGradient(point.x, point.y - columnHeight, point.x, point.y);
    column.addColorStop(0, `rgba(255,228,245,${alpha})`);
    column.addColorStop(0.42, `rgba(232,121,249,${alpha * 0.96})`);
    column.addColorStop(1, `rgba(126,34,206,${alpha * 0.20})`);
    ctx.strokeStyle = column;
    ctx.lineWidth = Math.max(4, radius * 0.72);
    ctx.lineCap = "round";
    ctx.beginPath();
    ctx.moveTo(point.x, point.y);
    ctx.lineTo(point.x - radius * 0.42, point.y - columnHeight);
    ctx.stroke();

    ctx.fillStyle = `rgba(251,207,232,${alpha})`;
    ctx.beginPath();
    ctx.ellipse(
      point.x - radius * 0.42,
      point.y - columnHeight,
      radius * 0.78,
      radius * 0.46,
      -0.25,
      0,
      Math.PI * 2
    );
    ctx.fill();
  });

  ctx.restore();

  ctx.strokeStyle = "rgba(34,211,238,0.32)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(centerX - farWidth / 2, floorTop);
  ctx.lineTo(centerX + farWidth / 2, floorTop);
  ctx.lineTo(centerX + nearWidth / 2, floorBottom);
  ctx.lineTo(centerX - nearWidth / 2, floorBottom);
  ctx.closePath();
  ctx.stroke();

  const robotX = centerX;
  const robotY = floorBottom - 26;
  ctx.fillStyle = "rgba(52,211,153,0.95)";
  ctx.shadowColor = "rgba(52,211,153,0.85)";
  ctx.shadowBlur = 20;
  ctx.beginPath();
  ctx.arc(robotX, robotY, 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.shadowBlur = 0;
  ctx.strokeStyle = "rgba(236,253,245,0.92)";
  ctx.lineWidth = 2;
  ctx.stroke();

  ctx.strokeStyle = "rgba(45,212,191,0.26)";
  ctx.lineWidth = 2;
  ctx.setLineDash([10, 12]);
  ctx.beginPath();
  ctx.moveTo(robotX, robotY);
  ctx.lineTo(centerX - farWidth / 2, floorTop);
  ctx.moveTo(robotX, robotY);
  ctx.lineTo(centerX + farWidth / 2, floorTop);
  ctx.stroke();
  ctx.setLineDash([]);

  const vignette = ctx.createRadialGradient(centerX, height * 0.5, height * 0.12, centerX, height * 0.5, height * 0.86);
  vignette.addColorStop(0, "rgba(0,0,0,0)");
  vignette.addColorStop(1, "rgba(0,0,0,0.46)");
  ctx.fillStyle = vignette;
  ctx.fillRect(0, 0, width, height);
};
