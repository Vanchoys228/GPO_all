import { createElement, useRef } from "react";
import {
  BatteryCharging,
  Gauge,
  Layers3,
  MapPin,
  Play,
  Radio,
  Route,
  Settings2,
  Shield,
  SlidersHorizontal,
  Trash2,
  Upload,
  X,
  Zap,
} from "lucide-react";
import { ALGORITHM_OPTIONS, TASK_OPTIONS } from "../../lib/routeAlgorithms";
import {
  DEFAULT_POINT_TASK,
  POINT_TASKS,
  ROUTE_CLEARANCE_MARGIN,
  SAFE_POINT_MARGIN,
} from "../../lib/zonePlanner";
import {
  SURFACE_PROFILE_OPTIONS,
  describeSurfaceRuntime,
} from "../../lib/energyModel";


const panelCls = "rounded-lg border border-slate-200 bg-white p-3 shadow-sm";
const inputCls =
  "h-10 w-full rounded-md border border-slate-200 bg-white px-3 text-sm text-slate-900 outline-none transition focus:border-teal-500 focus:ring-2 focus:ring-teal-100";
const subtleButtonCls =
  "inline-flex h-10 items-center justify-center gap-2 rounded-md border border-slate-200 bg-white px-3 text-sm font-semibold text-slate-700 shadow-sm transition hover:bg-slate-50";
const pointRowCls =
  "flex items-center justify-between gap-3 rounded-md border border-slate-200 bg-white px-3 py-2 transition hover:border-sky-200 hover:bg-sky-50";
const dangerButtonCls =
  "inline-flex h-9 items-center justify-center gap-2 rounded-md border border-rose-200 bg-rose-50 px-3 text-xs font-semibold text-rose-700 transition hover:bg-rose-100";

const formatSeconds = (seconds) => {
  if (!Number.isFinite(seconds) || seconds <= 0) return "0 c";
  if (seconds < 60) return `${seconds.toFixed(1)} c`;
  return `${(seconds / 60).toFixed(1)} мин`;
};

const parseLooseInput = (rawValue, fallback) => {
  const normalized = String(rawValue ?? "")
    .trim()
    .replace(",", ".");
  if (!normalized) return fallback;
  const parsed = Number(normalized);
  return Number.isFinite(parsed) ? parsed : fallback;
};

function SectionTitle({ icon: Icon, title, action }) {
  return (
    <div className="flex items-center justify-between gap-3">
      <div className="flex min-w-0 items-center gap-2">
        {createElement(Icon, { className: "shrink-0 text-slate-500", size: 17 })}
        <h3 className="truncate text-sm font-semibold text-slate-950">{title}</h3>
      </div>
      {action}
    </div>
  );
}

export default function PlannerLeftSidebar({
  activeTab = "route",
  routeTaskKey,
  onRouteTaskChange,
  algorithmKey,
  onAlgorithmChange,
  algorithmFields,
  selectedAlgorithmParams,
  onAlgorithmParamChange,
  isOptimizing,
  onOptimizeRoute,
  onSendRoute,
  onAddRandomObstacle,
  onClearObstacles,
  onImportGraph,
  visitEntries,
  chargeEntries,
  plannedVisitEntries,
  expandedPoint,
  hoveredPointIndex,
  onToggleExpandedPoint,
  onHoverPoint,
  onDeletePoint,
  onUpdatePointTask,
  onClearVisitPoints,
  onClearChargePoints,
  batteryRangeInput,
  onBatteryRangeChange,
  onBatteryRangeBlur,
  cruiseSpeedMps,
  cruiseSpeedInput,
  onCruiseSpeedChange,
  onCruiseSpeedBlur,
  payloadKg,
  payloadInput,
  onPayloadChange,
  onPayloadBlur,
  routeEnergyStats,
}) {
  const fileInputRef = useRef(null);
  const plannedVisitLookup = new Map(plannedVisitEntries.map((entry) => [entry.index, entry]));

  const handleImportClick = () => {
    fileInputRef.current?.click();
  };

  const handleFileChange = (event) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (loadEvent) => {
      try {
        const rawText = String(loadEvent.target?.result ?? "");
        const parsed = JSON.parse(rawText);
        onImportGraph?.(parsed, file.name);
      } catch (error) {
        console.error("Graph import failed", error);
        window.alert("Не удалось импортировать граф: проверьте JSON-файл.");
      } finally {
        event.target.value = "";
      }
    };
    reader.readAsText(file);
  };

  const renderVisitPoints = () => (
    <div className={panelCls}>
      <SectionTitle
        icon={MapPin}
        title="Точки посещения"
        action={<span className="text-xs font-semibold text-slate-500">{visitEntries.length} шт.</span>}
      />
      <div className="mt-3 space-y-2">
        {visitEntries.map((entry) => {
          const expanded = expandedPoint === entry.index;
          const plannedEntry = plannedVisitLookup.get(entry.index);
          return (
            <div key={entry.index}>
              <div
                className={`${pointRowCls} ${hoveredPointIndex === entry.index ? "border-sky-300 bg-sky-50" : ""}`}
                onClick={() => onToggleExpandedPoint(expanded ? null : entry.index)}
                onMouseEnter={() => onHoverPoint(entry.index)}
                onMouseLeave={() => onHoverPoint(null)}
              >
                <div className="min-w-0">
                  <div className="truncate text-sm font-semibold text-slate-950">
                    V{entry.order} ({entry.point.x.toFixed(2)}, {entry.point.y.toFixed(2)})
                  </div>
                  {plannedEntry?.adjusted && <div className="mt-1 text-xs text-amber-700">Безопасная позиция</div>}
                </div>
                <button
                  type="button"
                  title="Удалить точку"
                  aria-label="Удалить точку"
                  onClick={(event) => { event.stopPropagation(); onDeletePoint(entry.index); }}
                  className="inline-flex h-8 w-8 shrink-0 items-center justify-center rounded-md border border-rose-200 bg-rose-50 p-0 text-rose-700 hover:bg-rose-100"
                >
                  <X size={15} />
                </button>
              </div>
              {expanded && (
                <label className="mt-2 block rounded-md border border-slate-200 bg-slate-50 p-3">
                  <div className="mb-1 text-xs font-medium text-slate-500">Операция</div>
                  <select
                    className={inputCls}
                    value={entry.point.task || DEFAULT_POINT_TASK}
                    onChange={(event) => onUpdatePointTask(entry.index, event.target.value)}
                  >
                    {POINT_TASKS.map((task) => <option key={task} value={task}>{task}</option>)}
                  </select>
                </label>
              )}
            </div>
          );
        })}
        {!visitEntries.length && <div className="text-sm text-slate-500">Точек посещения пока нет.</div>}
      </div>
      <button type="button" onClick={onClearVisitPoints} className={`${dangerButtonCls} mt-3 w-full`}>
        <Trash2 size={15} /> Очистить маршрутные
      </button>
    </div>
  );

  const renderChargePoints = () => (
    <div className={panelCls}>
      <SectionTitle
        icon={BatteryCharging}
        title="Станции зарядки"
        action={<span className="text-xs font-semibold text-slate-500">{chargeEntries.length} шт.</span>}
      />
      <div className="mt-3 space-y-2">
        {chargeEntries.map((entry) => (
          <div
            key={entry.index}
            className={`${pointRowCls} ${hoveredPointIndex === entry.index ? "border-amber-300 bg-amber-50" : ""}`}
            onMouseEnter={() => onHoverPoint(entry.index)}
            onMouseLeave={() => onHoverPoint(null)}
          >
            <div className="truncate text-sm font-semibold text-slate-950">
              C{entry.order} ({entry.point.x.toFixed(2)}, {entry.point.y.toFixed(2)})
            </div>
            <button
              type="button"
              title="Удалить зарядку"
              aria-label="Удалить зарядку"
              onClick={() => onDeletePoint(entry.index)}
              className="inline-flex h-8 w-8 shrink-0 items-center justify-center rounded-md border border-rose-200 bg-rose-50 p-0 text-rose-700 hover:bg-rose-100"
            >
              <X size={15} />
            </button>
          </div>
        ))}
        {!chargeEntries.length && <div className="text-sm text-slate-500">Станций зарядки пока нет.</div>}
      </div>
      <button type="button" onClick={onClearChargePoints} className={`${dangerButtonCls} mt-3 w-full`}>
        <Trash2 size={15} /> Очистить зарядки
      </button>
    </div>
  );

  const renderRouteTab = () => (
    <div className="space-y-3">
      <div className={panelCls}>
        <SectionTitle icon={Route} title="Планирование маршрута" />
        <p className="mt-2 text-sm leading-5 text-slate-600">
          Добавляйте точки посещения, станции зарядки и ограничивающие зоны. Теперь расчёт учитывает покрытие пола, массу груза и заданную скорость.
        </p>
        <div className="mt-3 rounded-md border border-teal-200 bg-teal-50 px-3 py-2 text-xs leading-5 text-teal-800">
          Маршрут строится с зазором {ROUTE_CLEARANCE_MARGIN.toFixed(2)} м от контура, а целевые точки держатся минимум на {SAFE_POINT_MARGIN.toFixed(2)} м от запретной зоны.
        </div>
      </div>

      <div className={panelCls}>
        <SectionTitle icon={Zap} title="Действия" />
        <div className="mt-3 grid grid-cols-2 gap-2">
          <button
            type="button"
            onClick={onOptimizeRoute}
            disabled={isOptimizing}
            className={`inline-flex h-11 items-center justify-center gap-2 rounded-md px-3 text-sm font-semibold text-white shadow-sm transition ${
              isOptimizing ? "cursor-wait bg-orange-400" : "bg-orange-600 hover:bg-orange-700"
            }`}
          >
            <Play size={16} />
            {isOptimizing ? "Строим" : "Построить"}
          </button>
          <button
            type="button"
            onClick={onSendRoute}
            disabled={isOptimizing}
            className={`inline-flex h-11 items-center justify-center gap-2 rounded-md px-3 text-sm font-semibold text-white shadow-sm transition ${
              isOptimizing ? "cursor-not-allowed bg-teal-400" : "bg-teal-700 hover:bg-teal-800"
            }`}
          >
            <Radio size={16} />
            Отправить
          </button>
          <button
            type="button"
            onClick={onAddRandomObstacle}
            disabled={isOptimizing}
            className={`col-span-2 inline-flex h-10 items-center justify-center gap-2 rounded-md border px-3 text-sm font-semibold shadow-sm transition ${
              isOptimizing
                ? "cursor-not-allowed border-sky-100 bg-sky-50 text-sky-400"
                : "border-sky-200 bg-sky-50 text-sky-800 hover:bg-sky-100"
            }`}
          >
            <Shield size={16} />
            Добавить случайное препятствие
          </button>
          <button
            type="button"
            onClick={onClearObstacles}
            disabled={isOptimizing}
            className={`col-span-2 inline-flex h-10 items-center justify-center gap-2 rounded-md border px-3 text-sm font-semibold shadow-sm transition ${
              isOptimizing
                ? "cursor-not-allowed border-rose-100 bg-rose-50 text-rose-300"
                : "border-rose-200 bg-rose-50 text-rose-700 hover:bg-rose-100"
            }`}
          >
            <Trash2 size={16} />
            Очистить препятствия
          </button>
        </div>
      </div>

      {renderVisitPoints()}

      <div className={panelCls}>
        <SectionTitle icon={Upload} title="Импорт графа" />
        <p className="mt-2 text-xs leading-5 text-slate-600">
          Загрузите JSON с точками, зарядками и ограничивающими зонами.
        </p>
        <button type="button" onClick={handleImportClick} className={`${subtleButtonCls} mt-3 w-full`}>
          <Upload size={16} />
          Загрузить граф
        </button>
        <input
          ref={fileInputRef}
          type="file"
          accept=".json,application/json"
          onChange={handleFileChange}
          className="hidden"
        />
      </div>

    </div>
  );

  const renderAlgorithmTab = () => (
    <div className="space-y-3">
      <div className={panelCls}>
        <SectionTitle icon={Settings2} title="Задача и алгоритм" />
        <label className="mt-3 block">
          <div className="mb-1 text-xs font-medium text-slate-500">Задача маршрута</div>
          <select
            className={inputCls}
            value={routeTaskKey}
            onChange={(event) => onRouteTaskChange(event.target.value)}
          >
            {TASK_OPTIONS.map((task) => (
              <option key={task.key} value={task.key}>
                {task.label}
              </option>
            ))}
          </select>
        </label>
        <label className="mt-3 block">
          <div className="mb-1 text-xs font-medium text-slate-500">Алгоритм</div>
          <select
            className={inputCls}
            value={algorithmKey}
            onChange={(event) => onAlgorithmChange(event.target.value)}
          >
            {ALGORITHM_OPTIONS.map((option) => (
              <option key={option.key} value={option.key}>
                {option.label}
              </option>
            ))}
          </select>
        </label>
      </div>

      <div className={panelCls}>
        <SectionTitle icon={SlidersHorizontal} title="Параметры" />
        <div className="mt-3 space-y-3">
          {algorithmFields.map((field) => (
            <label key={field.key} className="block">
              <div className="mb-1 text-xs font-medium text-slate-500">{field.label}</div>
              <input
                type="number"
                min={field.min}
                max={field.max}
                step={field.step}
                value={selectedAlgorithmParams[field.key]}
                className={inputCls}
                onChange={(event) => onAlgorithmParamChange(field, event.target.value)}
              />
            </label>
          ))}
        </div>
      </div>
    </div>
  );

  const renderEnergyTab = () => (
    <div className="space-y-3">
      <div className={panelCls}>
        <SectionTitle icon={BatteryCharging} title="Входные данные" />
        <label className="mt-3 block">
          <div className="mb-1 text-xs font-medium text-slate-500">Запас хода, энерго-ед.</div>
          <input
            type="text"
            inputMode="numeric"
            value={batteryRangeInput}
            className={inputCls}
            onChange={(event) => onBatteryRangeChange(event.target.value)}
            onBlur={onBatteryRangeBlur}
          />
        </label>
        <div className="mt-3 grid grid-cols-2 gap-2">
          <label className="block">
            <div className="mb-1 text-xs font-medium text-slate-500">Скорость, м/с</div>
            <input
              type="text"
              inputMode="decimal"
              value={cruiseSpeedInput}
              className={inputCls}
              onChange={(event) => onCruiseSpeedChange(event.target.value)}
              onBlur={onCruiseSpeedBlur}
            />
          </label>
          <label className="block">
            <div className="mb-1 text-xs font-medium text-slate-500">Груз, кг</div>
            <input
              type="text"
              inputMode="decimal"
              value={payloadInput}
              className={inputCls}
              onChange={(event) => onPayloadChange(event.target.value)}
              onBlur={onPayloadBlur}
            />
          </label>
        </div>
      </div>

      <div className={panelCls}>
        <SectionTitle icon={Layers3} title="Покрытия карты" />
        <div className="mt-3 space-y-2">
          {SURFACE_PROFILE_OPTIONS.map((profile) => {
            const runtime = describeSurfaceRuntime(profile, {
              speedMps: parseLooseInput(cruiseSpeedInput, cruiseSpeedMps),
              payloadKg: parseLooseInput(payloadInput, payloadKg),
            });
            return (
              <div key={profile.key} className="rounded-md border border-slate-200 bg-slate-50 px-3 py-2 text-xs">
                <div className="flex items-center gap-2">
                  <span
                    className="inline-block h-3 w-3 rounded-sm border border-slate-400"
                    style={{ background: profile.fill }}
                  />
                  <span className="font-semibold text-slate-900">{profile.label}</span>
                </div>
                <div className="mt-1 text-slate-600">
                  скорость {runtime.requestedSpeedMps.toFixed(2)} м/с, факт {runtime.effectiveSpeedMps.toFixed(2)}
                </div>
                <div className="text-slate-600">
                  расход x{runtime.energyMultiplier.toFixed(2)}, база x{profile.energyPerMeter.toFixed(2)}
                </div>
              </div>
            );
          })}
        </div>
      </div>

      {renderChargePoints()}

      <div className={panelCls}>
        <SectionTitle icon={Gauge} title="Расчет маршрута" />
        <div className="mt-3 grid grid-cols-2 gap-2">
          <div className="rounded-md border border-slate-200 bg-slate-50 p-3">
            <div className="text-xs text-slate-500">Энергия</div>
            <div className="mt-1 text-lg font-bold text-amber-700">
              {routeEnergyStats.routeEnergy.toFixed(1)}
            </div>
          </div>
          <div className="rounded-md border border-slate-200 bg-slate-50 p-3">
            <div className="text-xs text-slate-500">Время</div>
            <div className="mt-1 text-lg font-bold text-teal-700">
              {formatSeconds(routeEnergyStats.estimatedTimeSec)}
            </div>
          </div>
          <div className="rounded-md border border-slate-200 bg-slate-50 p-3">
            <div className="text-xs text-slate-500">Лимит</div>
            <div className="mt-1 text-lg font-bold text-slate-950">
              {routeEnergyStats.limitingMaxSpeedMps.toFixed(2)}
            </div>
          </div>
          <div className="rounded-md border border-slate-200 bg-slate-50 p-3">
            <div className="text-xs text-slate-500">Риск</div>
            <div className="mt-1 text-lg font-bold text-rose-700">
              {(routeEnergyStats.averageSlipRisk * 100).toFixed(1)}%
            </div>
          </div>
        </div>
      </div>
    </div>
  );


  const content = {
    route: renderRouteTab(),
    algorithm: renderAlgorithmTab(),
    energy: renderEnergyTab(),
  }[activeTab];

  return (
    <div className="h-full min-h-0 overflow-auto p-4">{content}</div>
  );
}
