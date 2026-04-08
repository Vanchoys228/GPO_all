import { ALGORITHM_OPTIONS, TASK_OPTIONS } from "../../lib/routeAlgorithms";
import {
  POINT_KIND_OPTIONS,
  ROUTE_CLEARANCE_MARGIN,
  SAFE_POINT_MARGIN,
} from "../../lib/zonePlanner";
import {
  SURFACE_PROFILE_OPTIONS,
  describeSurfaceRuntime,
} from "../../lib/energyModel";

const cardCls =
  "rounded-2xl bg-white/95 backdrop-blur border border-stone-200 shadow-[0_18px_40px_rgba(15,23,42,0.06)] p-4";
const inputCls =
  "w-full rounded-xl border border-stone-300 bg-white px-3 py-2 text-sm text-stone-900 shadow-sm transition focus:border-sky-400 focus:outline-none focus:ring-2 focus:ring-sky-100";
const subtleButtonCls =
  "rounded-xl border border-stone-300 bg-stone-100 px-3 py-2 text-sm font-semibold text-stone-800 shadow-sm transition hover:bg-stone-200 hover:border-stone-400";
const neutralButtonCls =
  "rounded-xl border border-slate-300 bg-slate-800 px-3 py-2 text-sm font-semibold text-white shadow-sm transition hover:bg-slate-900";

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

export default function PlannerLeftSidebar({
  activePointKind,
  onActivePointKindChange,
  onClearVisitPoints,
  onClearChargePoints,
  onClearLimitPoints,
  routeTaskKey,
  onRouteTaskChange,
  algorithmKey,
  onAlgorithmChange,
  status,
  energyWarning,
  routeBlocked,
  algorithmFields,
  selectedAlgorithmParams,
  onAlgorithmParamChange,
  isOptimizing,
  onOptimizeRoute,
  onSendRoute,
  onClearAll,
  hasRoute,
  routeLength,
  visitCount,
  chargeCount,
  zoneCount,
  polygonCount,
  adjustedVisitCount,
  activeZoneName,
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
  return (
    <aside className="w-[310px] overflow-auto border-r border-stone-200 bg-gradient-to-b from-stone-100 via-white to-slate-100 p-4 space-y-4 xl:w-[330px]">
      <div className={cardCls}>
        <div className="text-[11px] uppercase tracking-[0.22em] text-stone-500 mb-2">
          Карта маршрута
        </div>
        <h2 className="text-2xl font-bold leading-tight">Планировщик маршрута робота</h2>
        <p className="mt-2 text-sm text-stone-600">
          Добавляйте точки посещения, станции зарядки и ограничивающие зоны.
          Теперь расчёт учитывает покрытие пола, массу груза и заданную скорость.
        </p>
      </div>

      <div className={cardCls}>
        <div className="flex items-center justify-between gap-3">
          <h3 className="text-sm font-semibold">Легенда и обзор</h3>
          <span
            className={`inline-flex rounded-full px-2.5 py-1 text-[11px] font-semibold ${
              routeBlocked ? "bg-rose-100 text-rose-700" : "bg-teal-100 text-teal-700"
            }`}
          >
            {routeBlocked ? "маршрут задевает контур" : "маршрут свободен"}
          </span>
        </div>

        <div className="mt-3 grid grid-cols-1 gap-2 text-xs text-stone-700">
          <div className="flex items-center gap-2">
            <span className="inline-block h-3 w-3 rounded-full bg-rose-600" />
            <span>Точки посещения</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="inline-block h-3 w-3 rounded-full bg-amber-500" />
            <span>Станции зарядки</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="inline-block h-3 w-3 rotate-45 bg-blue-600" />
            <span>Ограничивающая зона</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="inline-block h-3 w-3 rounded-full bg-yellow-500" />
            <span>Автосдвинутая безопасная точка</span>
          </div>
          <div className="flex items-center gap-2">
            <span
              className={`inline-block h-[3px] w-8 rounded-full ${
                routeBlocked ? "bg-rose-600" : "bg-teal-700"
              }`}
            />
            <span>Линия маршрута</span>
          </div>
        </div>

        <div className="mt-4 grid grid-cols-2 gap-2 text-xs">
          <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2">
            <div className="text-stone-500">Маршрутных точек</div>
            <div className="mt-1 text-base font-semibold text-stone-900">{visitCount}</div>
          </div>
          <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2">
            <div className="text-stone-500">Станций зарядки</div>
            <div className="mt-1 text-base font-semibold text-stone-900">{chargeCount}</div>
          </div>
          <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2">
            <div className="text-stone-500">Запретных зон</div>
            <div className="mt-1 text-base font-semibold text-stone-900">{zoneCount}</div>
          </div>
          <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2">
            <div className="text-stone-500">Готовых контуров</div>
            <div className="mt-1 text-base font-semibold text-stone-900">{polygonCount}</div>
          </div>
          <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2 col-span-2">
            <div className="text-stone-500">Точек с автосдвигом</div>
            <div className="mt-1 text-base font-semibold text-stone-900">{adjustedVisitCount}</div>
          </div>
        </div>

        <div className="mt-3 rounded-xl border border-stone-200 bg-stone-50 px-3 py-2 text-xs text-stone-700">
          Активная зона: <span className="font-semibold text-stone-900">{activeZoneName}</span>
        </div>
      </div>

      <div className={cardCls}>
        <div className="text-xs text-stone-600 mb-2">Режим добавления точки</div>
        <div className="grid grid-cols-2 gap-2">
          {POINT_KIND_OPTIONS.map((option) => {
            const active = activePointKind === option.key;
            return (
              <button
                key={option.key}
                onClick={() => onActivePointKindChange(option.key)}
                className={`rounded-2xl border px-3 py-3 text-left transition ${
                  active
                    ? `${option.border} ${option.bg} shadow-sm`
                    : "border-stone-200 bg-white hover:bg-stone-50"
                }`}
              >
                <div
                  className={`text-xs uppercase tracking-[0.2em] ${
                    active ? option.text : "text-stone-400"
                  }`}
                >
                  {option.shortLabel}
                </div>
                <div className="mt-1 text-sm font-semibold">{option.label}</div>
              </button>
            );
          })}
        </div>
        <div className="mt-3 grid grid-cols-1 gap-2 text-xs">
          <button onClick={onClearVisitPoints} className={subtleButtonCls}>
            Очистить маршрутные
          </button>
          <button onClick={onClearChargePoints} className={subtleButtonCls}>
            Очистить зарядки
          </button>
          <button onClick={onClearLimitPoints} className={subtleButtonCls}>
            Очистить зоны
          </button>
        </div>
      </div>

      <div className={cardCls}>
        <div className="text-xs text-stone-600 mb-1">Задача маршрута</div>
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
        <div className="text-xs text-stone-600 mt-3 mb-1">Алгоритм</div>
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
        {status && (
          <div className={`mt-3 text-sm ${routeBlocked ? "text-rose-700" : "text-emerald-700"}`}>
            {status}
          </div>
        )}
      </div>

      <div className={cardCls}>
        <h3 className="text-sm font-semibold mb-3">Энергия и динамика</h3>
        <label>
          <div className="text-xs text-stone-600 mb-1">Запас хода (энерго-ед.)</div>
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
          <label className="col-span-1">
            <div className="text-xs text-stone-600 mb-1">Скорость, м/с</div>
            <input
              type="text"
              inputMode="decimal"
              value={cruiseSpeedInput}
              className={inputCls}
              onChange={(event) => onCruiseSpeedChange(event.target.value)}
              onBlur={onCruiseSpeedBlur}
            />
          </label>
          <label className="col-span-1">
            <div className="text-xs text-stone-600 mb-1">Масса груза, кг</div>
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

        <div className="mt-3 rounded-xl border border-stone-200 bg-stone-50 px-3 py-2 text-xs text-stone-700 space-y-1">
          <div>Энергия маршрута: <span className="font-semibold">{routeEnergyStats.routeEnergy.toFixed(1)}</span></div>
          <div>Время прохода: <span className="font-semibold">{formatSeconds(routeEnergyStats.estimatedTimeSec)}</span></div>
          <div>Лимит по покрытию: <span className="font-semibold">{routeEnergyStats.limitingMaxSpeedMps.toFixed(2)} м/с</span></div>
          <div>Риск проскальзывания: <span className="font-semibold">{(routeEnergyStats.averageSlipRisk * 100).toFixed(1)}%</span></div>
        </div>

        <div className="mt-3 rounded-xl border border-sky-200 bg-sky-50 px-3 py-2 text-xs text-sky-800">
          Карта покрытий влияет на расход, допустимую скорость и риски в поворотах.
        </div>

        {energyWarning && (
          <div className="mt-3 rounded-xl border border-rose-200 bg-rose-50 px-3 py-2 text-xs text-rose-700">
            {energyWarning}
          </div>
        )}
      </div>

      <div className={cardCls}>
        <h3 className="text-sm font-semibold mb-3">Покрытия карты</h3>
        <div className="space-y-2 text-xs">
          {SURFACE_PROFILE_OPTIONS.map((profile) => (
            (() => {
              const runtime = describeSurfaceRuntime(profile, {
                speedMps: parseLooseInput(cruiseSpeedInput, cruiseSpeedMps),
                payloadKg: parseLooseInput(payloadInput, payloadKg),
              });
              return (
                <div
                  key={profile.key}
                  className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2"
                >
                  <div className="flex items-center gap-2">
                    <span
                      className="inline-block h-3 w-3 rounded-full border border-stone-400"
                      style={{ background: profile.fill }}
                    />
                    <span className="font-semibold text-stone-800">{profile.label}</span>
                  </div>
                  <div className="mt-1 text-stone-600">
                    скорость {runtime.requestedSpeedMps.toFixed(2)} м/с (лимит {runtime.surfaceMaxSpeedMps.toFixed(2)}, факт{" "}
                    {runtime.effectiveSpeedMps.toFixed(2)})
                  </div>
                  <div className="text-stone-600">
                    расход x{runtime.energyMultiplier.toFixed(2)} (база x{profile.energyPerMeter.toFixed(2)})
                  </div>
                </div>
              );
            })()
          ))}
        </div>
      </div>

      <div className={cardCls}>
        <h3 className="text-sm font-semibold mb-3">Параметры алгоритма</h3>
        <div className="space-y-3">
          {algorithmFields.map((field) => (
            <label key={field.key}>
              <div className="text-xs text-stone-600 mb-1">{field.label}</div>
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

      <div className={cardCls}>
        <div className="mb-3 rounded-xl border border-teal-200 bg-teal-50 px-3 py-2 text-xs text-teal-800">
          Маршрут строится с зазором {ROUTE_CLEARANCE_MARGIN.toFixed(2)} м от контура,
          а целевые точки держатся минимум на {SAFE_POINT_MARGIN.toFixed(2)} м от запретной зоны.
        </div>
        <button
          onClick={onOptimizeRoute}
          disabled={isOptimizing}
          className={`w-full h-11 rounded-xl text-white font-semibold transition ${
            isOptimizing ? "bg-orange-400 cursor-wait" : "bg-orange-600 hover:bg-orange-700"
          }`}
        >
          {isOptimizing ? "Строим маршрут..." : "Построить маршрут"}
        </button>
        <button
          onClick={onSendRoute}
          disabled={isOptimizing}
          className={`mt-2 w-full h-11 rounded-xl text-white font-semibold transition ${
            isOptimizing ? "bg-emerald-400 cursor-not-allowed" : "bg-emerald-600 hover:bg-emerald-700"
          }`}
        >
          Отправить маршрут
        </button>
        <button onClick={onClearAll} className={`mt-2 w-full h-11 ${neutralButtonCls}`}>
          Очистить всё
        </button>
        {hasRoute && (
          <p className="mt-3 text-sm">
            Длина маршрута: <b>{routeLength.toFixed(2)} м</b>
          </p>
        )}
      </div>
    </aside>
  );
}
