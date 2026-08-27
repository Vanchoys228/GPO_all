# Полный аудит проекта GPO

## Резюме

Проект находится в рабочем промежуточном состоянии: frontend, Node.js bridge и
нативный Webots-контроллер собираются, автоматические JavaScript-тесты проходят.
При этом архитектурная миграция ещё не завершена: сервисный слой появился, но
часть границ остаётся внутрипроцессной, а `youbot_web.c` всё ещё является главным
монолитом проекта.

Текущая оценка готовности:

| Область | Состояние | Готовность |
|---|---|---:|
| Frontend-декомпозиция | Основные hooks, models, services и sections выделены | 85% |
| Route / Planning / Telemetry service layer | Сервисы и отдельные entrypoint существуют | 75% |
| Shared contracts | Базовые envelope-контракты реализованы | 55% |
| Robot Control | Много доменных модулей выделено, coordinator остаётся большим | 65% |
| Gateway | Настоящий единый Gateway отсутствует | 15% |
| Независимые процессы | Три Node.js процесса запускаются отдельно | 55% |
| Интеграционные проверки | Есть bridge smoke test, нет полного Webots E2E | 35% |
| Docker | Контейнеризован только объединённый bridge | 20% |
| Webots в контейнере | Не реализован | 0% |

## Что было проверено

Успешно выполнены:

- `npm run lint`;
- `npm run test`: 51 тестовый файл, 136 тестов;
- `npm run build`;
- `npm run test:bridge`;
- `webots/controllers/youbot_web/build_youbot_web.bat`.

Production-сборка frontend выдаёт предупреждение о bundle размером около 724 КБ
и устаревшей базе Browserslist. Это не останавливает сборку, но требует отдельной
оптимизации.

Полный `run_controller_tests.bat` нельзя считать зелёным: текущая регистрация
новых Webots-модулей и include paths неполна. Это отдельная P1-проблема ниже.

## Найденные проблемы

### P1 — обязательные исправления до дальнейшего разделения процессов

#### P1.1. Полный C-тестовый контур больше не соответствует production-сборке

Файл `webots/controllers/youbot_web/run_controller_tests.bat` не включает
`controller_webots_simulation.c`. При этом `Makefile` и `build_youbot_web.bat`
уже включают его. Следовательно, тестовая и production-конфигурации расходятся.

Дополнительно `controller_webots_adapter_test.c` включает adapter header, который
теперь транзитивно требует Webots headers, но тестовый скрипт не передаёт путь
`%WEBOTS_HOME%/include/controller/c`.

Контрольный запуск останавливается уже на
`controller_avoidance_lifecycle_test.c` с ошибкой отсутствующего
`webots/robot.h`. При этом batch-скрипт после строки `compile failed` возвращает
внешнему процессу код `0`, поэтому CI способен ошибочно отметить запуск зелёным.

Риск: production-контроллер собирается, а полный тестовый прогон не способен
надёжно проверить ту же композицию исходников.

#### P1.2. Telemetry WebSocket может упасть на некорректном JSON

В `bridge/servers/telemetry-server.cjs` входящее сообщение разбирается прямым
`JSON.parse(text)` внутри async callback без локального `try/catch`.

Риск: повреждённый или несовместимый кадр от Webots Adapter способен породить
необработанное отклонение и нарушить передачу телеметрии всем клиентам.

Требование: безопасный parser, явная ошибка контракта, тест некорректного JSON и
сохранение работоспособности соединения после отклонённого сообщения.

#### P1.3. Текущая рабочая папка не является Git-репозиторием

Актуальные изменения находятся в `C:/GPO_ne_monolit`, где отсутствует `.git`.
Реальная ветка `refactor/monolith-decomposition`, отслеживающая
`gpo-all/dev/nimmwee`, находится в `C:/GPO_publish_monolith`.

Риск: случайное копирование generated-файлов, потеря изменений или коммит не в
ту ветку. Синхронизация должна выполняться только после фильтрации артефактов.

### P2 — архитектурные блокеры микросервисной цели

#### P2.1. `youbot_web.c` остаётся монолитом

Файл содержит около 3 тысяч строк и одновременно владеет:

- lifecycle контроллера;
- глобальным runtime-состоянием;
- камерой и виртуальным рендером;
- LiDAR trace и картами;
- dynamic/surface zones;
- runtime obstacles;
- mapping survey orchestration;
- route reload;
- telemetry snapshot;
- Webots Supervisor effects.

Доменные алгоритмы уже существенно вынесены, но точка композиции всё ещё слишком
велика. Цель: 500–800 строк тонкого Webots coordinator после выделения Camera,
Mapping, Simulation и Robot Control runtime adapters.

#### P2.2. Gateway Service отсутствует

Frontend напрямую знает три адреса и три порта: Telemetry, Route и Planning.
`ws-bridge.cjs` запускает local stack, но не является Gateway-прокси.

Риск: изменение топологии требует пересборки frontend; единая авторизация,
валидация, correlation IDs, service availability и routing отсутствуют.

#### P2.3. Robot Control ещё не является самостоятельным сервисом

Навигационные модули существуют, но исполняются внутри процесса Webots.
Отсутствуют сетевые `SensorFrame -> MotionCommand` контракты и safe-stop при
потере связи.

До вынесения процесса необходимо измерить latency и оставить fallback-режим,
поскольку управление колёсами чувствительно к задержкам.

#### P2.4. Shared contracts неполны

Реализованы envelope, RouteCommand, PlanningRequest/Result, TelemetryEvent и
MotionCommand. Не завершены заявленные RobotState, SensorFrame, Route,
ServiceError, health/readiness и idempotency contracts.

Валидация в основном структурная; payload schemas и единый формат ошибок пока
не обеспечены.

#### P2.5. Нет защиты команд от повторного выполнения

`requestId` присутствует в envelope, но Route Service не хранит обработанные ID.
После reconnect/retry команда может быть применена повторно.

Особенно опасны `spawn_obstacle`, mapping survey и активация маршрута.

#### P2.6. Health checks есть не у всех сервисов

Planning имеет `/health`, Route и Telemetry не имеют самостоятельных
health/readiness endpoints. Local Stack не проверяет готовность дочерних
компонентов.

Без этого невозможно корректно настроить Compose startup order и диагностику.

#### P2.7. Entry points не управляют graceful shutdown самостоятельно

Отдельные `run-*-service.cjs` запускают сервер, но не устанавливают единый
обработчик SIGINT/SIGTERM. Graceful shutdown реализован только в общем bridge.

Это критично для рестартов контейнеров и освобождения портов.

### P3 — качество, структура и эксплуатация

#### P3.1. Docker отражает старую объединённую архитектуру

`compose.yaml` содержит один `bridge` container с тремя портами. Отдельных
образов Route, Planning, Telemetry, Gateway, Robot Control и Webots нет.

Это допустимо как временный compatibility-container, но не как конечная схема.

#### P3.2. Webots GUI в контейнере не спроектирован до уровня запуска

Не определены:

- базовый образ и версия Webots;
- headless/Xvfb режим;
- browser GUI через noVNC/websockify;
- лицензирование и системные библиотеки;
- GPU/software rendering;
- volume для world/state;
- health check симуляции.

#### P3.3. Generated-файлы присутствуют в рабочем дереве

В `C:/GPO_ne_monolit` находятся `.obj`, `.exe`, test JSON/CSV и runtime-файлы
`web_state`. Их нельзя переносить в Git-клон. `.gitignore` необходимо расширить
на `*.obj` и тестовые артефакты в корне.

#### P3.4. Остались крупные frontend/native файлы

- `src/pages/Dashboard.jsx`: около 500 строк;
- `src/lib/energyModel.js`: более 400 строк;
- `src/lib/chargingPlanner.js`: более 400 строк;
- `native/src/common.cpp`: более 400 строк;
- `controller_survey_geometry.c`: более 700 строк.

Это не все являются архитектурными монолитами. Разделять их следует по реальной
ответственности, а не только по количеству строк.

#### P3.5. Frontend bundle превышает рекомендуемый размер Vite

Основной JS chunk около 724 КБ. Возможные причины: dashboard, XLSX и тяжёлые
планировочные модули загружаются сразу.

Рекомендация: lazy route для Dashboard, динамический import XLSX и manual chunks.

#### P3.6. Отсутствует полный локальный E2E

Bridge smoke test проверяет solver, но не проверяет полный поток:

`Frontend -> Gateway -> Route -> Webots -> Robot Control -> Telemetry -> Frontend`.

Нет автоматических сценариев отключения Planning, Route, Telemetry и Robot
Control, а также проверки safe stop.

## Положительные результаты аудита

- Frontend уже разделён на page, components, hooks, model и services.
- Planning, Route и Telemetry имеют service-layer модули и отдельные entrypoint.
- Общий local stack вынесен из `ws-bridge.cjs`.
- Native solver отделён от HTTP transport.
- Robot Control имеет большое количество чистых тестируемых C-модулей.
- Webots motor, pose и sensor API вынесены из основной оркестрации.
- Есть единый coordinate contract и первые versioned service envelopes.

## Рекомендуемый порядок исправления

1. Восстановить полный C-тестовый контур.
2. Защитить Telemetry Server от некорректных сообщений.
3. Завершить Simulation/Camera/Mapping adapters и сократить `youbot_web.c`.
4. Завершить shared contracts, idempotency и service errors.
5. Реализовать Gateway и health/readiness всех процессов.
6. Добавить process-level и Webots E2E проверки.
7. Очистить generated-файлы и структуру.
8. Только после зелёного local E2E перейти к контейнерам и Webots GUI.

## Критерий готовности к Docker

Проект готов к контейнеризации только когда каждый сервис запускается отдельно,
имеет health/readiness, использует versioned contracts, корректно переживает
отключение соседнего сервиса, а полный локальный E2E с настоящим Webots проходит
без ручного исправления файлов и портов.
