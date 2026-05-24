import { useState } from "react";
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
import CollapsibleSection from "./CollapsibleSection";

const MAP_POINT_KIND_OPTIONS = POINT_KIND_OPTIONS.filter((option) => option.key !== "limit");

const inputCls =
  "w-full rounded-xl border border-stone-300 bg-white px-3 py-2.5 text-sm text-stone-900 shadow-sm transition focus:border-sky-400 focus:outline-none focus:ring-2 focus:ring-sky-100";
const subtleButtonCls =
  "rounded-xl border border-stone-300 bg-stone-100 px-3 py-2.5 text-sm font-semibold text-stone-900 shadow-sm transition hover:bg-stone-200 hover:border-stone-400";
const clearAllButtonCls =
  "rounded-xl border border-rose-300 bg-white px-3 py-2 text-sm font-semibold text-rose-800 shadow-sm transition hover:border-rose-400 hover:bg-rose-50";

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

function SurfaceProfileCompactRow({ profile, cruiseSpeedInput, cruiseSpeedMps, payloadInput, payloadKg }) {
  const [detail, setDetail] = useState(false);
  const runtime = describeSurfaceRuntime(profile, {
    speedMps: parseLooseInput(cruiseSpeedInput, cruiseSpeedMps),
    payloadKg: parseLooseInput(payloadInput, payloadKg),
  });

  return (
    <button
      type="button"
      onClick={() => setDetail((d) => !d)}
      className="flex w-full flex-col rounded-xl border border-stone-200 bg-stone-50 px-3 py-2 text-left transition hover:border-stone-300 hover:bg-white"
    >
      <div className="flex items-center gap-2 text-sm text-stone-900">
        <span
          className="inline-block h-3 w-3 shrink-0 rounded-full border border-stone-400"
          style={{ background: profile.fill }}
        />
        <span className="min-w-0 flex-1 font-semibold text-stone-800">{profile.label}</span>
        <span className="shrink-0 tabular-nums text-stone-600">
          ×{runtime.energyMultiplier.toFixed(2)}
        </span>
        <span className="shrink-0 text-xs text-stone-600">{detail ? "▼" : "▶"}</span>
      </div>
      {detail ? (
        <div className="mt-2 space-y-1 border-t border-stone-200 pt-2 text-sm leading-relaxed text-stone-800">
          <div>
            скорость {runtime.requestedSpeedMps.toFixed(2)} м/с (лимит {runtime.surfaceMaxSpeedMps.toFixed(2)},
            факт {runtime.effectiveSpeedMps.toFixed(2)})
          </div>
          <div>
            расход ×{runtime.energyMultiplier.toFixed(2)} (база ×{profile.energyPerMeter.toFixed(2)})
          </div>
        </div>
      ) : null}
    </button>
  );
}

export default function PlannerLeftSidebar({
  onCollapse,
  activePointKind,
  onActivePointKindChange,
  onClearVisitPoints,
  onClearChargePoints,
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
    <aside className="flex h-full min-h-0 w-[310px] min-w-[310px] max-w-[310px] flex-none flex-col overflow-hidden border-r border-stone-200 bg-gradient-to-b from-stone-100 via-white to-slate-100 xl:w-[330px] xl:min-w-[330px] xl:max-w-[330px]">
      <div className="flex shrink-0 items-start justify-between gap-2 border-b border-stone-200 bg-white px-3 py-2">
        <div className="min-w-0 pt-0.5">
          <div className="text-xs font-medium uppercase tracking-[0.16em] text-stone-700">Планировщик</div>
          <div className="truncate text-base font-bold text-stone-900">Маршрут робота</div>
        </div>
        {onCollapse ? (
          <button
            type="button"
            title="Свернуть панель"
            aria-label="Свернуть левую панель"
            onClick={onCollapse}
            className="shrink-0 rounded-lg border border-stone-300 bg-white px-2 py-1.5 text-stone-600 shadow-sm transition hover:border-stone-400 hover:bg-stone-50"
          >
            <span className="text-lg leading-none" aria-hidden>
              ‹
            </span>
          </button>
        ) : null}
      </div>

      <div className="min-h-0 min-w-0 max-w-full flex-1 overflow-x-hidden overflow-y-auto overscroll-contain p-4 space-y-3 [scrollbar-gutter:stable]">
        <p className="text-sm leading-relaxed text-stone-800">
          Маршрутные точки и зарядки — кликом по карте; ограничивающие зоны — в правой панели.
        </p>

        <CollapsibleSection
          sectionId="left.legend"
          defaultOpen
          title="Легенда и обзор"
          description="Цвета на карте и счётчики объектов"
        >
          <div className="flex items-center justify-between gap-3">
            <span className="text-sm font-medium text-stone-800">Статус контура</span>
            <span
              className={`inline-flex rounded-full px-2.5 py-1 text-xs font-semibold ${
                routeBlocked ? "bg-rose-100 text-rose-700" : "bg-teal-100 text-teal-700"
              }`}
            >
              {routeBlocked ? "маршрут задевает контур" : "маршрут свободен"}
            </span>
          </div>

          <div className="mt-3 grid grid-cols-1 gap-2 text-sm text-stone-800">
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

          <div className="mt-4 grid grid-cols-2 gap-2 text-sm">
            <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2">
              <div className="text-stone-700">Маршрутных точек</div>
              <div className="mt-1 text-base font-semibold text-stone-800">{visitCount}</div>
            </div>
            <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2">
              <div className="text-stone-700">Станций зарядки</div>
              <div className="mt-1 text-base font-semibold text-stone-800">{chargeCount}</div>
            </div>
            <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2">
              <div className="text-stone-700">Запретных зон</div>
              <div className="mt-1 text-base font-semibold text-stone-800">{zoneCount}</div>
            </div>
            <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2">
              <div className="text-stone-700">Готовых контуров</div>
              <div className="mt-1 text-base font-semibold text-stone-800">{polygonCount}</div>
            </div>
            <div className="rounded-xl border border-stone-200 bg-stone-50 px-3 py-2 col-span-2">
              <div className="text-stone-700">Точек с автосдвигом</div>
              <div className="mt-1 text-base font-semibold text-stone-800">{adjustedVisitCount}</div>
            </div>
          </div>

          <div className="mt-3 rounded-xl border border-stone-200 bg-stone-50 px-3 py-2 text-sm leading-relaxed text-stone-800">
            Активная зона: <span className="font-semibold text-stone-900">{activeZoneName}</span>
          </div>
        </CollapsibleSection>

        <CollapsibleSection
          sectionId="left.points"
          defaultOpen
          title="Режим точки и очистка"
          description="Маршрутные точки и зарядки; зоны — в правой панели"
        >
          <div className="grid grid-cols-2 gap-2">
            {MAP_POINT_KIND_OPTIONS.map((option) => {
              const active = activePointKind === option.key;
              return (
                <button
                  key={option.key}
                  type="button"
                  onClick={() => onActivePointKindChange(option.key)}
                  className={`rounded-2xl border px-3 py-3 text-left transition ${
                    active
                      ? `${option.border} ${option.bg} shadow-sm`
                      : "border-stone-200 bg-white hover:bg-stone-50"
                  }`}
                >
                  <div
                    className={`text-xs font-semibold uppercase tracking-[0.14em] ${
                      active ? option.text : "text-stone-600"
                    }`}
                  >
                    {option.shortLabel}
                  </div>
                  <div
                    className={`mt-1 text-sm font-semibold leading-snug ${active ? option.text : "text-stone-800"}`}
                  >
                    {option.label}
                  </div>
                </button>
              );
            })}
          </div>
          <div className="mt-3 grid grid-cols-1 gap-2">
            <button type="button" onClick={onClearVisitPoints} className={subtleButtonCls}>
              Очистить маршрутные
            </button>
            <button type="button" onClick={onClearChargePoints} className={subtleButtonCls}>
              Очистить зарядки
            </button>
          </div>
        </CollapsibleSection>

        <CollapsibleSection
          sectionId="left.routeAlgo"
          defaultOpen
          title="Задача и алгоритм"
          description="Тип маршрута и способ расчёта"
        >
          <div className="text-sm font-medium text-stone-800 mb-1">Задача маршрута</div>
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
          <div className="text-sm font-medium text-stone-800 mt-3 mb-1">Алгоритм</div>
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
          {status ? (
            <div className={`mt-3 text-sm leading-relaxed ${routeBlocked ? "text-rose-800" : "text-emerald-800"}`}>
              {status}
            </div>
          ) : null}
        </CollapsibleSection>

        <CollapsibleSection
          sectionId="left.energy"
          defaultOpen
          title="Энергия и динамика"
          description="Запас хода, скорость, масса и оценка расхода"
        >
          <label>
            <div className="text-sm font-medium text-stone-800 mb-1">Запас хода (энерго-ед.)</div>
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
              <div className="text-sm font-medium text-stone-800 mb-1">Скорость, м/с</div>
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
              <div className="text-sm font-medium text-stone-800 mb-1">Масса груза, кг</div>
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

          <div className="mt-3 rounded-xl border border-stone-200 bg-stone-50 px-3 py-2 text-sm text-stone-900 space-y-1.5 leading-relaxed">
            <div>
              Энергия маршрута:{" "}
              <span className="font-semibold">{routeEnergyStats.routeEnergy.toFixed(1)}</span>
            </div>
            <div>
              Время прохода:{" "}
              <span className="font-semibold">{formatSeconds(routeEnergyStats.estimatedTimeSec)}</span>
            </div>
            <div>
              Лимит по покрытию:{" "}
              <span className="font-semibold">{routeEnergyStats.limitingMaxSpeedMps.toFixed(2)} м/с</span>
            </div>
            <div>
              Риск проскальзывания:{" "}
              <span className="font-semibold">{(routeEnergyStats.averageSlipRisk * 100).toFixed(1)}%</span>
            </div>
          </div>

          <div className="mt-3 rounded-xl border border-sky-200 bg-sky-50 px-3 py-2 text-sm leading-relaxed text-sky-950">
            Карта покрытий влияет на расход, допустимую скорость и риски в поворотах.
          </div>

          {energyWarning ? (
            <div className="mt-3 rounded-xl border border-rose-200 bg-rose-50 px-3 py-2 text-sm leading-relaxed text-rose-900">
              {energyWarning}
            </div>
          ) : null}
        </CollapsibleSection>

        <CollapsibleSection
          sectionId="left.surfaces"
          defaultOpen={false}
          title="Покрытия карты"
          description="Кратко по типам пола; подробности по строке"
        >
          <div className="space-y-2">
            {SURFACE_PROFILE_OPTIONS.map((profile) => (
              <SurfaceProfileCompactRow
                key={profile.key}
                profile={profile}
                cruiseSpeedInput={cruiseSpeedInput}
                cruiseSpeedMps={cruiseSpeedMps}
                payloadInput={payloadInput}
                payloadKg={payloadKg}
              />
            ))}
          </div>
        </CollapsibleSection>

        <CollapsibleSection
          sectionId="left.algoParams"
          defaultOpen
          title="Параметры алгоритма"
          description="Численные настройки выбранного метода"
        >
          <div className="space-y-3">
            {algorithmFields.map((field) => (
              <label key={field.key}>
                <div className="text-sm font-medium text-stone-800 mb-1">{field.label}</div>
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
        </CollapsibleSection>
      </div>

      <div className="min-w-0 max-w-full shrink-0 overflow-x-hidden border-t border-stone-200 bg-white px-4 py-3 shadow-[0_-6px_20px_rgb(15_23_42/0.12)]">
        <div className="mb-3 rounded-xl border border-teal-200 bg-teal-50 px-3 py-2 text-sm leading-relaxed text-teal-950">
          Зазор маршрута {ROUTE_CLEARANCE_MARGIN.toFixed(2)} м от контура; точки не ближе{" "}
          {SAFE_POINT_MARGIN.toFixed(2)} м к запретной зоне.
        </div>
        <button
          type="button"
          onClick={onOptimizeRoute}
          disabled={isOptimizing}
          className={`w-full h-11 rounded-xl text-white font-semibold transition ${
            isOptimizing ? "bg-orange-400 cursor-wait" : "bg-orange-600 hover:bg-orange-700"
          }`}
        >
          {isOptimizing ? "Строим маршрут..." : "Построить маршрут"}
        </button>
        <button
          type="button"
          onClick={onSendRoute}
          disabled={isOptimizing}
          className={`mt-2 w-full h-11 rounded-xl text-white font-semibold transition ${
            isOptimizing ? "bg-emerald-400 cursor-not-allowed" : "bg-emerald-600 hover:bg-emerald-700"
          }`}
        >
          Отправить маршрут
        </button>
        <button type="button" onClick={onClearAll} className={`mt-2 w-full h-11 ${clearAllButtonCls}`}>
          Очистить всё
        </button>
        {hasRoute ? (
          <p className="mt-2 text-center text-sm font-medium text-stone-900">
            Длина маршрута: <b>{routeLength.toFixed(2)} м</b>
          </p>
        ) : null}
      </div>
    </aside>
  );
}
