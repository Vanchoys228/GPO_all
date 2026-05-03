import { DEFAULT_POINT_TASK, POINT_KIND_META, POINT_TASKS } from "../../lib/zonePlanner";

const zonePanelCardCls =
  "rounded-[26px] bg-white/97 backdrop-blur border border-sky-100 shadow-[0_18px_50px_rgba(125,211,252,0.18)] p-4";
const zoneCardBaseCls =
  "rounded-2xl border p-3 bg-white shadow-[0_10px_24px_rgba(15,23,42,0.06)]";
const rowCls =
  "flex items-center justify-between gap-3 px-3 py-2 rounded-xl border border-stone-300 bg-white shadow-sm hover:bg-stone-50 hover:border-stone-400 cursor-pointer transition";
const selectCls =
  "w-full rounded-xl border border-stone-300 bg-white px-3 py-2 text-sm text-stone-900 shadow-sm transition focus:border-sky-400 focus:outline-none focus:ring-2 focus:ring-sky-100";
const zonePrimaryButtonCls =
  "rounded-md border border-sky-300 bg-sky-100 px-2.5 py-1.5 text-[11px] font-semibold text-sky-800 shadow-sm transition hover:bg-sky-200";
const zoneNeutralButtonCls =
  "rounded-md border border-stone-300 bg-stone-100 px-2.5 py-1.5 text-[11px] font-semibold text-stone-800 shadow-sm transition hover:bg-stone-200";
const zoneDangerButtonCls =
  "rounded-md border border-rose-300 bg-rose-50 px-2.5 py-1.5 text-[11px] font-semibold text-rose-700 shadow-sm transition hover:bg-rose-100";

export default function PlannerRightSidebar({
  activeZone,
  activeZoneName,
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
  telemetryWsUp,
  routeWsUp,
  solverApiUp,
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

  return (
    <aside className="w-[330px] overflow-auto border-l border-sky-100 bg-gradient-to-b from-sky-50 via-white to-cyan-50 p-4 space-y-4 xl:w-[350px]">
      <div className={zonePanelCardCls}>
        <div className="flex items-start justify-between gap-3">
          <div>
            <div className="text-xs uppercase tracking-[0.18em] text-sky-700">Активная зона</div>
            <div className="mt-1 text-lg font-semibold text-slate-900">{activeZoneName}</div>
          </div>
          <div className="text-right text-xs text-slate-500">
            <div>{activeZone?.closed ? "Замкнута" : "Открыта"}</div>
            <div className="mt-1">Точек: {activeZone?.points.length || 0}</div>
          </div>
        </div>
        <div className="mt-3 text-xs text-slate-600 space-y-1">
          <div>В открытую зону можно добавлять и двигать точки.</div>
          <div>Замкнутая зона участвует в расчете безопасного маршрута.</div>
          <div>Точки внутри замкнутого контура автоматически выносятся в безопасную позицию.</div>
        </div>
      </div>

      <div className={zonePanelCardCls}>
        <div className="flex items-center justify-between gap-3 mb-3">
          <div>
            <h3 className="text-sm font-semibold text-slate-900">Ограничивающие зоны</h3>
            <div className="mt-1 text-xs text-slate-500">Управление контурами и их статусом</div>
          </div>
          <button
            onClick={onCreateZone}
            className="rounded-xl bg-sky-700 px-3 py-2 text-xs font-semibold text-white shadow-sm transition hover:bg-sky-800"
          >
            Новая зона
          </button>
        </div>
        <div className="space-y-2">
          {zoneEntries.map((zone) => {
            const active = zone.id === activeLimitZoneId;
            return (
              <div
                key={zone.id}
                className={`${zoneCardBaseCls} ${
                  active
                    ? "border-sky-300 bg-gradient-to-br from-sky-50 to-white"
                    : "border-sky-100 bg-white"
                }`}
              >
                <div className="flex items-start justify-between gap-3">
                  <button
                    className="flex-1 min-w-0 rounded-xl px-2 py-1 text-left transition hover:bg-sky-50"
                    onClick={() => onSelectZone(zone.id)}
                  >
                    <div className="flex items-center gap-2">
                      <span className={`inline-block w-3 h-3 rounded-full ${zone.color.badge}`} />
                      <span className="text-sm font-semibold text-slate-900">{zone.name}</span>
                      <span
                        className={`inline-flex rounded-full px-2 py-0.5 text-[10px] font-semibold ${
                          zone.closed ? "bg-emerald-100 text-emerald-700" : "bg-amber-100 text-amber-700"
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
                </div>
                <div className="mt-3 flex flex-wrap gap-2">
                  <button onClick={() => onToggleZoneClosed(zone.id)} className={zonePrimaryButtonCls}>
                    {zone.closed ? "Открыть" : "Замкнуть"}
                  </button>
                  <button onClick={() => onClearZone(zone.id)} className={zoneNeutralButtonCls}>
                    Очистить
                  </button>
                  <button onClick={() => onRemoveZone(zone.id)} className={zoneDangerButtonCls}>
                    Удалить
                  </button>
                </div>
              </div>
            );
          })}
        </div>
      </div>

      <div className={zonePanelCardCls}>
        <div className="flex items-center justify-between gap-3 mb-3">
          <h3 className="text-sm font-semibold">Точки посещения</h3>
          <span className="text-xs text-stone-500">{visitEntries.length} шт.</span>
        </div>
        <div className="space-y-2">
          {visitEntries.map((entry) => {
            const expanded = expandedPoint === entry.index;
            const hovered = hoveredPointIndex === entry.index;
            const plannedEntry = plannedVisitLookup.get(entry.index);

            return (
              <div key={entry.index}>
                <div
                  className={`${rowCls} ${hovered ? "border-sky-300 bg-sky-50 shadow-sm" : ""}`}
                  onClick={() => onToggleExpandedPoint(expanded ? null : entry.index)}
                  onMouseEnter={() => onHoverPoint(entry.index)}
                  onMouseLeave={() => onHoverPoint(null)}
                >
                  <div className="min-w-0">
                    <div className="text-sm font-medium">
                      V{entry.order} ({entry.point.x.toFixed(2)}, {entry.point.y.toFixed(2)})
                    </div>
                    <div className="inline-flex mt-1 px-2 py-0.5 rounded-full border text-[11px] bg-rose-50 border-rose-200 text-rose-700">
                      {POINT_KIND_META.visit.label}
                    </div>
                    {plannedEntry?.adjusted && (
                      <div className="inline-flex mt-1 ml-2 px-2 py-0.5 rounded-full border text-[11px] bg-amber-50 border-amber-200 text-amber-700">
                        Автосдвиг
                      </div>
                    )}
                  </div>
                  <button
                    onClick={(event) => {
                      event.stopPropagation();
                      onDeletePoint(entry.index);
                    }}
                    className="flex items-center justify-center w-7 h-7 rounded-md border border-red-300 text-red-600 bg-red-50 hover:bg-red-600 hover:text-white transition"
                  >
                    x
                  </button>
                </div>
                {expanded && (
                  <div className="mt-2 ml-2 p-3 rounded-xl border border-stone-200 bg-white space-y-2">
                    <div className="text-sm">
                      x: {entry.point.x.toFixed(4)}, y: {entry.point.y.toFixed(4)}
                    </div>
                    {plannedEntry?.adjusted && (
                      <div className="text-xs text-amber-700">
                        Безопасная точка маршрута: x={plannedEntry.plannedPoint.x.toFixed(4)}, y=
                        {plannedEntry.plannedPoint.y.toFixed(4)}
                      </div>
                    )}
                    <label>
                      <div className="text-xs text-stone-500 mb-1">Операция</div>
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
            <div className="text-sm text-stone-500">Добавьте на карте точки, которые нужно посетить.</div>
          )}
        </div>
      </div>

      <div className={zonePanelCardCls}>
        <div className="flex items-center justify-between gap-3 mb-3">
          <h3 className="text-sm font-semibold">Станции зарядки</h3>
          <span className="text-xs text-stone-500">{chargeEntries.length} шт.</span>
        </div>
        <div className="space-y-2">
          {chargeEntries.map((entry) => (
            <div
              key={entry.index}
              className={`${rowCls} ${
                hoveredPointIndex === entry.index ? "border-amber-300 bg-amber-50 shadow-sm" : ""
              }`}
              onMouseEnter={() => onHoverPoint(entry.index)}
              onMouseLeave={() => onHoverPoint(null)}
            >
              <div className="min-w-0">
                <div className="text-sm font-medium">
                  C{entry.order} ({entry.point.x.toFixed(2)}, {entry.point.y.toFixed(2)})
                </div>
                <div className="inline-flex mt-1 px-2 py-0.5 rounded-full border text-[11px] bg-amber-50 border-amber-200 text-amber-700">
                  {POINT_KIND_META.charge.label}
                </div>
              </div>
              <button
                onClick={(event) => {
                  event.stopPropagation();
                  onDeletePoint(entry.index);
                }}
                className="flex items-center justify-center w-7 h-7 rounded-md border border-red-300 text-red-600 bg-red-50 hover:bg-red-600 hover:text-white transition"
              >
                x
              </button>
            </div>
          ))}
          {!chargeEntries.length && (
            <div className="text-sm text-stone-500">
              Добавьте станции зарядки, чтобы робот мог выполнять длинные маршруты.
            </div>
          )}
        </div>
      </div>

      <div className={zonePanelCardCls}>
        <h3 className="text-sm font-semibold mb-2">Контроль ограничений</h3>
        <div className="space-y-2 text-sm text-stone-700">
          <div>
            Точек внутри зон: <b>{visitsInsideLimitCount}</b>
          </div>
          <div>
            Контуров, готовых для обхода: <b>{polygonCount}</b>
          </div>
          <div>
            Точек с автосдвигом: <b>{adjustedVisitCount}</b>
          </div>
          <div>
            Маршрут пересекает контур: <b>{routeBlocked ? "да" : "нет"}</b>
          </div>
        </div>
        <div className="mt-3 text-xs text-stone-500 space-y-1">
          <div>В обходе участвуют только замкнутые зоны.</div>
          <div>Любую точку можно перетащить мышкой прямо на карте.</div>
          <div>Если безопасный обход построить нельзя, маршрут не отправляется.</div>
        </div>
      </div>

      <div className={zonePanelCardCls}>
        <h3 className="text-sm font-semibold mb-2">Телеметрия</h3>
        <div className="grid grid-cols-2 gap-2 text-sm">
          <div>x: {telemetry.x.toFixed(2)}</div>
          <div>y: {telemetry.y.toFixed(2)}</div>
          <div>z: {telemetry.z.toFixed(2)}</div>
          <div>yaw: {telemetry.yaw.toFixed(2)}</div>
          <div>lidar: {telemetry.perception?.lidar?.enabled ? "on" : "off"}</div>
          <div>hits: {telemetry.obstacleTrace?.length || 0}</div>
          <div>cells: {telemetry.obstacleMap?.cellCount || telemetry.obstacleMap?.cells?.length || 0}</div>
          <div>cell: {(telemetry.obstacleMap?.cellSize || 0.06).toFixed(2)} м</div>
        </div>
        <div className="mt-3 rounded-xl border border-cyan-200 bg-cyan-50/70 p-3">
          <div className="text-xs font-semibold uppercase tracking-[0.12em] text-cyan-800">
            Карта препятствий
          </div>
          <div className="mt-1 text-xs text-cyan-900">
            Постоянная карта строится из лидарных попаданий и рисуется поверх рабочей сетки.
          </div>
          <button
            onClick={onStartMappingSurvey}
            className="mt-3 w-full rounded-xl bg-emerald-700 px-3 py-2 text-xs font-semibold text-white shadow-sm transition hover:bg-emerald-800"
          >
            Запустить объезд карты
          </button>
          <button
            onClick={onExportMapImage}
            disabled={!telemetry.obstacleMap?.cells?.length}
            className={`mt-3 w-full rounded-xl px-3 py-2 text-xs font-semibold shadow-sm transition ${
              telemetry.obstacleMap?.cells?.length
                ? "bg-cyan-700 text-white hover:bg-cyan-800"
                : "cursor-not-allowed bg-cyan-100 text-cyan-400"
            }`}
          >
            Сохранить карту в PNG
          </button>
        </div>
        <div className="mt-3 text-xs text-stone-600">
          WS Telemetry:{" "}
          <span className={telemetryWsUp ? "text-emerald-700" : "text-red-600"}>
            {telemetryWsUp ? "connected" : "disconnected"}
          </span>
        </div>
        <div className="text-xs text-stone-600">
          WS Route:{" "}
          <span className={routeWsUp ? "text-emerald-700" : "text-red-600"}>
            {routeWsUp ? "connected" : "disconnected"}
          </span>
        </div>
        <div className="text-xs text-stone-600">
          Native Solver:{" "}
          <span className={solverApiUp ? "text-emerald-700" : "text-red-600"}>
            {solverApiUp ? "connected" : "disconnected"}
          </span>
        </div>
      </div>
    </aside>
  );
}
