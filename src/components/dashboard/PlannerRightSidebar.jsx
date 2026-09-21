import { createElement } from "react";
import {
  Download,
  Eye,
  EyeOff,
  Layers3,
  Map as MapIcon,
  MapPin,
  Plus,
  Radio,
  RefreshCw,
  Route,
  Shield,
  Trash2,
  X,
  Zap,
} from "lucide-react";
import {
  DEFAULT_POINT_TASK,
  POINT_KIND_META,
  POINT_TASKS,
} from "../../lib/zonePlanner";


const panelCls = "rounded-lg border border-slate-200 bg-white p-3 shadow-sm";
const rowCls =
  "flex items-center justify-between gap-3 rounded-md border border-slate-200 bg-white px-3 py-2 shadow-sm transition hover:border-sky-200 hover:bg-sky-50";
const selectCls =
  "h-10 w-full rounded-md border border-slate-200 bg-white px-3 text-sm text-slate-900 outline-none transition focus:border-teal-500 focus:ring-2 focus:ring-teal-100";
const primaryButtonCls =
  "inline-flex h-9 items-center justify-center gap-2 rounded-md border border-sky-200 bg-sky-50 px-3 text-xs font-semibold text-sky-800 shadow-sm transition hover:bg-sky-100";
const neutralButtonCls =
  "inline-flex h-9 items-center justify-center gap-2 rounded-md border border-slate-200 bg-white px-3 text-xs font-semibold text-slate-700 shadow-sm transition hover:bg-slate-50";
const dangerButtonCls =
  "inline-flex h-9 items-center justify-center gap-2 rounded-md border border-rose-200 bg-rose-50 px-3 text-xs font-semibold text-rose-700 shadow-sm transition hover:bg-rose-100";

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

function IconButton({
  label,
  children,
  onClick,
  tone = "border-slate-200 bg-white text-slate-700 hover:bg-slate-50",
}) {
  return (
    <button
      type="button"
      title={label}
      aria-label={label}
      onClick={onClick}
      className={`inline-flex h-8 w-8 shrink-0 items-center justify-center rounded-md border p-0 shadow-sm transition ${tone}`}
    >
      {children}
    </button>
  );
}

export default function PlannerRightSidebar({
  activeTab = "zones",
  onClearLimitPoints,
  activeLimitZoneId,
  zoneEntries,
  visitEntries,
  chargeEntries,
  plannedVisitEntries,
  expandedPoint,
  hoveredPointIndex,
  visitsInsideLimitCount,
  polygonCount,
  adjustedVisitCount,
  routeBlocked,
  telemetry,
  visibleLayers,
  onToggleLayer,
  routeLength,
  mappingSurveyMode,
  mappingSurveyModes,
  onMappingSurveyModeChange,
  onStartMappingSurvey,
  onExportMapImage,
  onCreateZone,
  onSelectZone,
  onToggleZoneClosed,
  onClearZone,
  onRemoveZone,
  onToggleExpandedPoint,
  onHoverPoint,
  onDeletePoint,
  onUpdatePointTask,
}) {
  const plannedVisitLookup = new Map(plannedVisitEntries.map((entry) => [entry.index, entry]));

  const renderZonesTab = () => (
    <div className="space-y-3">
      <div className={panelCls}>
        <SectionTitle
          icon={Shield}
          title="Ограничивающие зоны"
          action={
            <IconButton
              label="Новая зона"
              onClick={onCreateZone}
              tone="border-sky-200 bg-sky-600 text-white hover:bg-sky-700"
            >
              <Plus size={17} />
            </IconButton>
          }
        />
        <div className="mt-3 space-y-2">
          {zoneEntries.map((zone) => {
            const active = zone.id === activeLimitZoneId;
            return (
              <div
                key={zone.id}
                className={`rounded-md border p-3 shadow-sm ${
                  active ? "border-sky-300 bg-sky-50" : "border-slate-200 bg-white"
                }`}
              >
                <button
                  type="button"
                  className="w-full rounded-md p-0 text-left"
                  onClick={() => onSelectZone(zone.id)}
                >
                  <div className="flex items-center justify-between gap-2">
                    <div className="flex min-w-0 items-center gap-2">
                      <span className={`inline-block h-3 w-3 shrink-0 rounded-sm ${zone.color.badge}`} />
                      <span className="truncate text-sm font-semibold text-slate-950">{zone.name}</span>
                    </div>
                    <span
                      className={`shrink-0 rounded-md px-2 py-1 text-xs font-semibold ${
                        zone.closed ? "bg-teal-100 text-teal-800" : "bg-amber-100 text-amber-800"
                      }`}
                    >
                      {zone.closed ? "Замкнута" : "Открыта"}
                    </span>
                  </div>
                  <div className="mt-2 grid grid-cols-2 gap-2 text-xs text-slate-500">
                    <div>Точек: {zone.points.length}</div>
                    <div>{zone.points.length >= 3 ? "Контур готов" : "Нужно 3 точки"}</div>
                  </div>
                </button>
                <div className="mt-3 grid grid-cols-3 gap-2">
                  <button type="button" onClick={() => onToggleZoneClosed(zone.id)} className={primaryButtonCls}>
                    {zone.closed ? "Открыть" : "Замкнуть"}
                  </button>
                  <button type="button" onClick={() => onClearZone(zone.id)} className={neutralButtonCls}>
                    Очистить
                  </button>
                  <button type="button" onClick={() => onRemoveZone(zone.id)} className={dangerButtonCls}>
                    Удалить
                  </button>
                </div>
              </div>
            );
          })}
        </div>
        <div className="mt-3 space-y-1 rounded-md border border-sky-200 bg-sky-50 px-3 py-2 text-xs leading-5 text-sky-900">
          <div>В открытую зону можно добавлять и двигать точки.</div>
          <div>Замкнутая зона участвует в расчете безопасного маршрута.</div>
          <div>Точки внутри замкнутого контура автоматически выносятся в безопасную позицию.</div>
        </div>
      </div>

      <div className={panelCls}>
        <SectionTitle icon={Zap} title="Контроль ограничений" />
        <div className="mt-3 grid grid-cols-2 gap-2 text-xs">
          {[
            ["Внутри зон", visitsInsideLimitCount],
            ["Контуров", polygonCount],
            ["Автосдвигов", adjustedVisitCount],
            ["Пересечение", routeBlocked ? "да" : "нет"],
          ].map(([label, value]) => (
            <div key={label} className="rounded-md border border-slate-200 bg-slate-50 px-3 py-2">
              <div className="text-slate-500">{label}</div>
              <div className="mt-1 text-base font-bold text-slate-950">{value}</div>
            </div>
          ))}
        </div>
        <div className="mt-3 space-y-1 text-xs leading-5 text-slate-600">
          <div>В обходе участвуют только замкнутые зоны.</div>
          <div>Любую точку можно перетащить мышкой прямо на карте.</div>
          <div>Если безопасный обход построить нельзя, маршрут не отправляется.</div>
        </div>
      </div>

      <button type="button" onClick={onClearLimitPoints} className={`${dangerButtonCls} w-full`}>
        <Trash2 size={15} />
        Очистить зоны
      </button>
    </div>
  );

  const _renderVisitRows = () => (
    <div className="space-y-2">
      {visitEntries.map((entry) => {
        const expanded = expandedPoint === entry.index;
        const hovered = hoveredPointIndex === entry.index;
        const plannedEntry = plannedVisitLookup.get(entry.index);

        return (
          <div key={entry.index}>
            <div
              className={`${rowCls} ${hovered ? "border-sky-300 bg-sky-50" : ""}`}
              onClick={() => onToggleExpandedPoint(expanded ? null : entry.index)}
              onMouseEnter={() => onHoverPoint(entry.index)}
              onMouseLeave={() => onHoverPoint(null)}
            >
              <div className="min-w-0">
                <div className="truncate text-sm font-semibold text-slate-950">
                  V{entry.order} ({entry.point.x.toFixed(2)}, {entry.point.y.toFixed(2)})
                </div>
                <div className="mt-1 flex flex-wrap gap-1">
                  <span className="rounded-md border border-rose-200 bg-rose-50 px-2 py-0.5 text-[11px] text-rose-700">
                    {POINT_KIND_META.visit.label}
                  </span>
                  {plannedEntry?.adjusted && (
                    <span className="rounded-md border border-amber-200 bg-amber-50 px-2 py-0.5 text-[11px] text-amber-700">
                      Автосдвиг
                    </span>
                  )}
                </div>
              </div>
              <IconButton
                label="Удалить точку"
                onClick={(event) => {
                  event.stopPropagation();
                  onDeletePoint(entry.index);
                }}
                tone="border-rose-200 bg-rose-50 text-rose-700 hover:bg-rose-100"
              >
                <X size={15} />
              </IconButton>
            </div>
            {expanded && (
              <div className="mt-2 rounded-md border border-slate-200 bg-slate-50 p-3">
                <div className="text-sm text-slate-700">
                  x: {entry.point.x.toFixed(4)}, y: {entry.point.y.toFixed(4)}
                </div>
                {plannedEntry?.adjusted && (
                  <div className="mt-2 text-xs text-amber-700">
                    Безопасная точка: x={plannedEntry.plannedPoint.x.toFixed(4)}, y=
                    {plannedEntry.plannedPoint.y.toFixed(4)}
                  </div>
                )}
                <label className="mt-3 block">
                  <div className="mb-1 text-xs font-medium text-slate-500">Операция</div>
                  <select
                    className={selectCls}
                    value={entry.point.task || DEFAULT_POINT_TASK}
                    onChange={(event) => onUpdatePointTask(entry.index, event.target.value)}
                  >
                    {POINT_TASKS.map((task) => (
                      <option key={task} value={task}>
                        {task}
                      </option>
                    ))}
                  </select>
                </label>
              </div>
            )}
          </div>
        );
      })}
      {!visitEntries.length && (
        <div className="rounded-md border border-slate-200 bg-slate-50 px-3 py-2 text-sm text-slate-500">
          Точек посещения пока нет.
        </div>
      )}
    </div>
  );

  const _renderChargeRows = () => (
    <div className="space-y-2">
      {chargeEntries.map((entry) => (
        <div
          key={entry.index}
          className={`${rowCls} ${hoveredPointIndex === entry.index ? "border-amber-300 bg-amber-50" : ""}`}
          onMouseEnter={() => onHoverPoint(entry.index)}
          onMouseLeave={() => onHoverPoint(null)}
        >
          <div className="min-w-0">
            <div className="truncate text-sm font-semibold text-slate-950">
              C{entry.order} ({entry.point.x.toFixed(2)}, {entry.point.y.toFixed(2)})
            </div>
            <span className="mt-1 inline-flex rounded-md border border-amber-200 bg-amber-50 px-2 py-0.5 text-[11px] text-amber-700">
              {POINT_KIND_META.charge.label}
            </span>
          </div>
          <IconButton
            label="Удалить зарядку"
            onClick={(event) => {
              event.stopPropagation();
              onDeletePoint(entry.index);
            }}
            tone="border-rose-200 bg-rose-50 text-rose-700 hover:bg-rose-100"
          >
            <X size={15} />
          </IconButton>
        </div>
      ))}
      {!chargeEntries.length && (
        <div className="rounded-md border border-slate-200 bg-slate-50 px-3 py-2 text-sm text-slate-500">
          Станций зарядки пока нет.
        </div>
      )}
    </div>
  );


  const renderMappingTab = () => (
    <div className="space-y-3">
      <div className={panelCls}>
        <SectionTitle icon={Layers3} title="Слои карты" />
        <div className="mt-3 space-y-2">
          {[
            ["surfaces", "Покрытия", Layers3],
            ["zones", "Отображение зон", Shield],
            ["obstacleTrace", "Следы датчиков", MapPin],
            ["route", "Маршрут", Route],
          ].map(([key, label, Icon]) => (
            <button
              key={key}
              type="button"
              onClick={() => onToggleLayer(key)}
              className={`flex h-10 w-full items-center justify-between rounded-md border px-3 text-sm font-semibold transition ${
                visibleLayers[key]
                  ? "border-teal-200 bg-teal-50 text-teal-800"
                  : "border-slate-200 bg-white text-slate-500 hover:bg-slate-50"
              }`}
            >
              <span className="flex items-center gap-2">{createElement(Icon, { size: 16 })}{label}</span>
              {visibleLayers[key] ? <Eye size={16} /> : <EyeOff size={16} />}
            </button>
          ))}
        </div>
      </div>

      <div className={panelCls}>
        <SectionTitle icon={MapPin} title="Легенда карты" />
        <div className="mt-3 space-y-2 text-xs text-slate-700">
          {[
            ["rounded-full bg-rose-600", "Точки посещения"],
            ["rounded-full bg-amber-500", "Станции зарядки"],
            ["rotate-45 bg-blue-600", "Ограничивающая зона"],
            ["rounded-full bg-yellow-500", "Автосдвинутая безопасная точка"],
          ].map(([tone, label]) => (
            <div key={label} className="flex items-center gap-2">
              <span className={`inline-block h-3 w-3 shrink-0 ${tone}`} />
              <span>{label}</span>
            </div>
          ))}
          <div className="flex items-center gap-2">
            <span className={`inline-block h-[3px] w-8 shrink-0 rounded-full ${routeBlocked ? "bg-rose-600" : "bg-teal-700"}`} />
            <span>Линия маршрута</span>
          </div>
        </div>
      </div>

      <div className={panelCls}>
        <SectionTitle icon={MapIcon} title="Обзор карты" />
        <div className="mt-3 grid grid-cols-2 gap-2 text-xs">
          {[
            ["Маршрутных точек", visitEntries.length],
            ["Станций зарядки", chargeEntries.length],
            ["Запретных зон", zoneEntries.length],
            ["Готовых контуров", polygonCount],
            ["Точек с автосдвигом", adjustedVisitCount],
          ].map(([label, value], index) => (
            <div key={label} className={`rounded-md border border-slate-200 bg-slate-50 px-3 py-2 ${index === 4 ? "col-span-2" : ""}`}>
              <div className="text-slate-500">{label}</div>
              <div className="mt-1 text-base font-bold text-slate-950">{value}</div>
            </div>
          ))}
        </div>
        <div className="mt-3 grid grid-cols-2 gap-2 text-xs">
          <div className="rounded-md border border-slate-200 bg-slate-50 px-3 py-2">
            <div className="text-slate-500">Длина маршрута</div>
            <div className="mt-1 font-bold text-slate-950">{routeLength.toFixed(2)} м</div>
          </div>
          <div className="rounded-md border border-slate-200 bg-slate-50 px-3 py-2">
            <div className="text-slate-500">Ячеек карты</div>
            <div className="mt-1 font-bold text-slate-950">{telemetry.obstacleMap?.cellCount || telemetry.obstacleMap?.cells?.length || 0}</div>
          </div>
        </div>
        <label className="mt-3 block">
          <div className="mb-1 text-xs font-medium text-slate-500">Режим обследования</div>
          <select
            className={selectCls}
            data-testid="mapping-survey-mode"
            value={mappingSurveyMode}
            onChange={(event) => onMappingSurveyModeChange(event.target.value)}
          >
            {mappingSurveyModes.map((mode) => (
              <option key={mode.key} value={mode.key}>
                {mode.label}
              </option>
            ))}
          </select>
        </label>
        <button
          type="button"
          onClick={onStartMappingSurvey}
          className="mt-3 inline-flex h-10 w-full items-center justify-center gap-2 rounded-md bg-teal-700 px-3 text-sm font-semibold text-white shadow-sm transition hover:bg-teal-800"
        >
          <RefreshCw size={16} />
          {mappingSurveyMode === "double" ? "Двойной объезд" : "Обследование карты"}
        </button>
        <button
          type="button"
          onClick={onExportMapImage}
          disabled={!telemetry.obstacleMap?.cells?.length}
          className={`mt-2 inline-flex h-10 w-full items-center justify-center gap-2 rounded-md px-3 text-sm font-semibold shadow-sm transition ${
            telemetry.obstacleMap?.cells?.length
              ? "bg-cyan-700 text-white hover:bg-cyan-800"
              : "cursor-not-allowed bg-cyan-100 text-cyan-400"
          }`}
        >
          <Download size={16} />
          Сохранить PNG
        </button>
      </div>
    </div>
  );

  const content = {
    zones: renderZonesTab(),
    mapping: renderMappingTab(),
  }[activeTab];

  return (
    <div className="h-full min-h-0 overflow-auto p-4">{content}</div>
  );
}
