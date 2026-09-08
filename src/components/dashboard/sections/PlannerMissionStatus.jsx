const labels = {prepared:"Подготовлена, отправка не подтверждена",persisted:"Сохранена, ожидается контроллер",accepted:"Принята контроллером",running:"Выполняется",completed:"Завершена",failed:"Ошибка исполнения",cancelled:"Отменена"};
export default function PlannerMissionStatus({mission}) {
  if (!mission) return null;
  return <section className="rounded-xl border border-slate-200 bg-white p-3 text-sm" aria-label="Состояние миссии">
    <div className="font-semibold">Исполнение маршрута</div>
    <div role="status" className="mt-1">{labels[mission.status] || "Состояние неизвестно"}</div>
    <div className="mt-1 break-all text-xs text-slate-500">ID: {mission.missionId}</div>
    {mission.connectionError && <div className="mt-1 text-amber-700">Нет связи с сервисом. Показано последнее известное состояние.</div>}
  </section>;
}
