const labels = {prepared:"Подготовлена, отправка не подтверждена",persisted:"Сохранена, ожидается контроллер",accepted:"Принята контроллером",running:"Выполняется",cancelling:"Отмена запрошена, ожидается остановка",completed:"Завершена",failed:"Ошибка исполнения",cancelled:"Отменена"};
export default function PlannerMissionStatus({mission,onCancel}) {
  if (!mission) return null;
  return <section className="rounded-xl border border-slate-200 bg-white p-3 text-sm" aria-label="Состояние миссии">
    <div className="font-semibold">Исполнение маршрута</div>
    <div role="status" className="mt-1">{labels[mission.status] || "Состояние неизвестно"}</div>
    <div className="mt-1 break-all text-xs text-slate-500">ID: {mission.missionId}</div>
    {mission.connectionError && <div className="mt-1 text-amber-700">Нет связи с сервисом. Показано последнее известное состояние.</div>}
    {!mission.connectionError && mission.feedbackFresh === false && !["completed","cancelled","failed"].includes(mission.status) && <div className="mt-1 text-amber-700">Нет свежего подтверждения от контроллера.</div>}
    {onCancel && !["completed","cancelled","failed"].includes(mission.status) && <button type="button" className="mt-2 rounded border border-red-300 px-3 py-1 text-red-700 disabled:opacity-50" disabled={mission.status === "cancelling"} onClick={onCancel}>Отменить миссию</button>}
  </section>;
}
