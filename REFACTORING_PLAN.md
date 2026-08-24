# План декомпозиции GPO

Цель — постепенно превратить проект в модульный монолит и подготовить Webots к
контейнерному запуску без изменения рабочего сценария пользователя.

## Уже выполнено

- Создан отдельный модуль runtime-конфигурации bridge.
- Добавлены отдельные переменные `BRIDGE_BIND_HOST`, `WEB_STATE_DIR` и
  `SOLVER_PATH`.
- Проверка входных сообщений вынесена в `bridge/protocol`.
- Запуск и протокол native solver вынесены в `bridge/solver`.
- HTTP API solver вынесено в `bridge/servers`.
- Файловое состояние Webots вынесено в `bridge/artifacts`.
- Route WebSocket и telemetry WebSocket вынесены в отдельные серверы.
- Чтение и нормализация телеметрии вынесены в `bridge/telemetry`.
- `ws-bridge.cjs` оставлен точкой композиции и lifecycle (72 строки вместо 1109).
- Добавлены модульные и контрактные тесты новых границ.
- Добавлена переносимая CMake-сборка solver.
- Добавлен первый Dockerfile для bridge и Linux solver.
- Добавлен минимальный `compose.yaml` для режима с Webots на компьютере.
- Frontend-подключения вынесены в `useTelemetrySocket`, `useRouteSocket` и
  `useSolverHealth`.
- Синхронизация зон, покрытий и параметров движения с bridge вынесена в
  `usePlannerBridgeSync`.
- Учёт времени маршрута вынесен в `useRouteTiming`.
- Импорт графа, модель покрытий, энергетика и случайные препятствия вынесены в
  тестируемые модули `src/features/planner/model`.
- Отрисовка экспортируемой карты вынесена в отдельный service.
- `Dashboard.jsx` уменьшен до 1479 строк вместо 2318 без изменения UI-контракта.
- Декомпозиция `youbot_web.c` начата с независимых модулей
  `controller_drive`, `controller_io`, `controller_math`, `controller_motion_profile` и
  `controller_route`, `controller_runtime_command`, `controller_telemetry`,
  `controller_types`, `controller_zones`, `controller_lidar_math`,
  `controller_avoidance`, `controller_camera`; Windows build-скрипт собирает их
  вместе с основным контроллером.
- Общие структуры маршрута, зон, картографии, лидара и Mapping Survey, а также
  enum-типы lifecycle вынесены в `controller_types.c/.h`. Размеры массивов и
  числовые значения enum зафиксированы автономным тестом.
- Загрузка и нормализация профиля движения, включая расчёт runtime-ограничений
  скорости, вынесены из `youbot_web.c` без изменения формата
  `motion_profile.txt` и формул управления.
- Парсинг `route.csv`, преобразование heading из градусов в радианы и чтение
  времени изменения маршрута вынесены в `controller_route.c/.h`. Сохранена
  поддержка двух- и трёхколоночных строк, заголовков и комментариев.
- Чтение и нормализация `runtime_command.txt` вынесены в
  `controller_runtime_command.c/.h`. Границы поля передаются через публичную
  структуру аргументов вместо доступа к глобальным переменным контроллера.
- Разбор `limit_zones.txt` и `surface_zones.txt`, а также сравнение снимков зон
  вынесены в `controller_zones.c/.h`; создание Supervisor-нод осталось в
  основном контроллере.
- Формирование JSON и атомарная запись `robot_state.json` вынесены в
  `controller_telemetry.c/.h`. Основной контроллер теперь передаёт явно
  заполненную структуру снимка без доступа сериализатора к Webots API или
  глобальным переменным.
- Mecanum-преобразование скорости базы в четыре скорости колёс, насыщение и
  ограничения разгона/торможения вынесены в `controller_drive.c/.h`. Наличие
  Webots-моторов передаётся явной маской.
- Чистая лидарная математика — давление по дальности, фильтрация одиночных
  лучей по соседям, уверенность накопленной трассы и накопление
  `LidarObstacleContext` — вынесена в `controller_lidar_math.c/.h`. Разрешение
  лидара и пороги передаются явно, без доступа модуля к Webots API, зонам или
  глобальному состоянию. В адаптере остались чтение range image, расчёт мировых
  координат луча и классификация ожидаемых стен зон.
- Классификация ситуации локального уклонения из `LidarObstacleContext`, выбор
  стороны обхода и расчёт linear/angular-команды для прохода, поворота,
  движения вдоль препятствия и escape вынесены в `controller_avoidance.c/.h`.
  Пороговая конфигурация и ограничения скорости передаются явно; применение
  команды, статусы и переходы state machine остаются в основном контроллере.
- Режим, счётчики прогресса/застревания, опорные координаты и detour-точка
  объединены в `ControllerAvoidanceState`. Полный reset состояния вынесен в
  `controller_avoidance_state_reset`. Геометрия detour-точки вынесена в
  `controller_avoidance_set_detour`, а атомарное обновление progress/stuck,
  свободного пространства и достижения detour — в
  `controller_avoidance_update_progress`. Обе функции получают состояние явно;
  начало обхода выполняет `controller_avoidance_state_begin`. Все переходные
  avoidance-макросы удалены, state machine обращается к структуре явно.
- Анализ RGB-кадра реальной камеры, включая crop, sampling, классификацию тёплых
  пикселей, score, центр и fallback-дистанцию, вынесен в
  `controller_camera.c/.h`. Модуль получает пиксели через callback и не зависит
  от Webots SDK; чтение Webots image, лидарная коррекция дистанции и обновление
  карты остаются в адаптере.
- Геометрия camera obstacle/free-space map также вынесена в `controller_camera`:
  преобразование pose и смещения сенсора в мировые точки, range guards, margin,
  шаг луча и near-robot filter получают все параметры явно. Чтение Webots pose,
  хранение и объединение ячеек остаются в основном контроллере.
- Нормализация, хранение и объединение camera obstacle/free-space ячеек вынесены
  в `controller_camera_map.c/.h`. Модуль работает с переданными буферами и
  ёмкостями, сохраняет приоритет obstacle-ячеек и ограничение confidence; dirty
  state и файловая запись остаются в адаптере.
- Растровые примитивы виртуальной камеры — clamped pixel, clipped rectangle,
  Bresenham line, фон с перспективной сеткой и obstacle box — вынесены в
  `controller_camera_render.c/.h`. Lidar-derived scene, route overlay и запись
  изображения остаются в адаптере.
- Первые чистые примитивы Mapping Survey — расширение bounds, добавление точек и
  разбиение сегментов маршрута, сортировка координат и вычитание интервалов —
  вынесены в `controller_survey_geometry.c/.h` с явными ёмкостями и порогами.
- В тот же модуль вынесены grid indexing, преобразование индекса в мировую точку,
  flood fill связной области, boundary detection и RDP keep-marking. Заполнение
  grid из зон и карты, ordering контура и orchestration маршрута пока остаются в
  основном контроллере.
- Grid pathfinding и сборка boundary route также вынесены в
  `controller_survey_geometry`: сохранены восьмисвязный BFS, parent traversal,
  thinning каждого третьего узла, nearest-neighbor ordering, join limit, RDP и
  замыкание контура.
- Общие правила horizontal/vertical coverage вынесены в чистый survey-модуль:
  clipping по bounds, минимальная длина полосы, разворот порядка интервалов и
  выбор ближайшего snake endpoint. Zone/map interval queries и safe transitions
  остаются в адаптере.
- Девять Mapping Survey runtime-полей объединены в
  `ControllerMappingSurveyState`. Инициализация, route reset, подготовка survey,
  cooldown tick, начало и завершение obstacle scan вынесены в
  `controller_survey_state.c/.h`; навигационные side effects остались в адаптере.
- Периодическое расписание zone/route/motion/runtime-command/map/camera задач
  вынесено в `controller_lifecycle.c/.h`. Интервалы передаются явно, а Webots и
  файловые side effects остаются в основном контроллере.
- Порядок одного controller step вынесен в `controller_step.c/.h` через явную
  callback-таблицу. Exact-order тест выявил и исправил регрессию lifecycle-среза:
  camera-map write снова выполняется после camera perception/frame, как в
  исходном цикле.
- По результатам полного ревью исправлено восстановление длинных Mapping Survey
  путей: parent chain теперь допускает полный размер grid, обязательно достигает
  стартовой ячейки и не возвращает успешный усечённый переход при недостаточной
  ёмкости выходного массива.
- Приоритет camera obstacle над free-space теперь сохраняется независимо от
  порядка наблюдений: добавление obstacle атомарно удаляет совпадающую свободную
  ячейку.
- Для восемнадцати C-модулей добавлены автономные тесты, не требующие Webots SDK.
- После строгой сборки удалены подтверждённо неиспользуемые legacy-функции
  reflex avoidance, cross-track/track heading и проверки dynamic zone, а также
  три неиспользуемые локальные переменные.
- `youbot_web.c` содержит 4349 строк против исходных 5804; небольшой рост
  относительно прошлого среза связан с явной lifecycle-конфигурацией и читаемой
  маской периодических задач в `main`.

## Проверено на текущем этапе

- 54 JavaScript-теста проходят.
- ESLint и Vite production build проходят.
- Автономные тесты `controller_avoidance`, `controller_camera`,
  `controller_camera_map`, `controller_drive`, `controller_io`, `controller_math`,
  `controller_camera_render`, `controller_lidar_math`, `controller_lifecycle`,
  `controller_motion_profile`, `controller_route`, `controller_runtime_command`,
  `controller_survey_geometry`, `controller_survey_state`, `controller_step`,
  `controller_telemetry`, `controller_types` и `controller_zones` проходят с
  Webots MinGW GCC 14.2 и флагами `-Wall -Wextra -Werror`.
- Полный контроллер компилируется и линкуется установленным Webots MinGW GCC
  против `libController.a` с флагами `-Wall -Wextra -Werror`; найденное
  отсутствующее предварительное объявление lidar helper исправлено без изменения
  алгоритма.
- Native solver дополнительно собран локально статическим Webots MinGW `g++`,
  после чего `npm run test:bridge` проходит полностью.
- Штатные `npm run native:build` и `npm run webots:build` пока требуют ручной
  повторной проверки после установки Visual Studio 2022 C++ Build Tools:
  текущий `.bat` не находит `vcvars64.bat`.
- Docker CLI и Compose установлены, но Docker Desktop engine на текущей машине
  не запущен, поэтому `docker compose build bridge` остаётся ручной проверкой.
- `npm audit --omit=dev` сообщает о четырёх production-уязвимостях в цепочках
  `react-router`, `ws` и `xlsx`; для `xlsx` автоматического исправления нет.
  Обновление зависимостей следует проводить отдельным этапом с регрессионными
  тестами, а не одновременно с переносом C-кода.

## Следующие безопасные шаги

Подробный зафиксированный план финальной декомпозиции, архитектурной очистки,
Docker и сквозной проверки находится в `docs/PROJECT_CLEANUP_PLAN.md`.

1. Следующим этапом выполнить финальную frontend-декомпозицию `Dashboard.jsx`,
   `zonePlanner.js` и крупных sidebar-компонентов, затем провести общий аудит.
3. Разделить оставшуюся orchestration-логику `Dashboard.jsx` на hooks действий
   редактора, построения маршрута и импорта/экспорта.
4. Разделить крупные sidebar-компоненты на секции с неизменными props.
5. Разделить `zonePlanner.js` на координаты, геометрию, проверку полигонов,
   построение маршрута и Canvas-отрисовку.

## Отложено до завершения декомпозиции монолитов

- Любые дополнительные Docker-работы, включая bridge-образ и Webots headless.
- Linux-контейнер полного контроллера Webots.
- Reverse proxy и единая контейнерная точка входа.

## Правила совместимости

До завершения интеграционных тестов нельзя одновременно менять:

- номера портов;
- WebSocket-путь `/ui`;
- структуру route/telemetry JSON;
- формат файлов в `web_state`;
- координатный контракт;
- алгоритмы движения робота.

После каждого шага должны проходить:

```bash
npm test
npm run lint
npm run build
```

На Windows дополнительно проверяется полный сценарий с Webots GUI.

## Режимы запуска

### Текущий переходный режим

- Webots GUI работает на Windows.
- Bridge и Linux solver работают в Docker.
- `web_state` подключается в bridge как общий каталог.

```bash
docker compose up --build bridge
```

### Целевой автоматический режим

- Frontend работает через HTTP/reverse proxy.
- Bridge и solver работают в одном контейнере.
- Webots headless работает в отдельном контейнере.
- Bridge и Webots используют общий Docker volume.

Графический Webots остаётся основным режимом ручной разработки.
