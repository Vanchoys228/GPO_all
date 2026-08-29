# Monolith Audit and youBot Web Decomposition Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Завершить декомпозицию `youbot_web.c` до тонкого Webots composition root и отдельно зафиксировать остальные модули проекта, которым действительно требуется разбиение.

**Architecture:** Сохранить текущий синхронный цикл Webots и форматы файлов, но собрать оставшуюся логику в несколько крупных runtime-компонентов с явными input/output структурами. Чистый Robot Control получает `SensorFrame` и возвращает `MotionCommand`/effects; Webots API, файловый I/O и Supervisor effects остаются во внешних адаптерах.

**Tech Stack:** C11, Webots C API, MSVC/Makefile, Vitest, React 19, Node.js, C++ native solver.

---

## 1. Проверенное состояние на 2026-08-29

Аудит выполнен в `C:/GPO_nimmwee/.worktrees/controller-test-runner`, ветка
`codex/controller-test-runner`, commit `e8b37a2`. Этот worktree совпадает с
`origin/dev/nimmwee` и на 60 коммитов опережает основной checkout
`C:/GPO_nimmwee` (`ff870d5`).

Проверки:

- `npm run lint` — PASS;
- `npm run build` — PASS, но основной JS chunk 727.68 KB;
- `npm run test:webots` — PASS, 69 C-тестов;
- `npm run webots:build` — PASS;
- `npm test` — FAIL: 6 устаревших assertion в
  `webots/controllerBuild.test.js`; 171 из 177 тестов проходят.

Шесть падений проверяют, что вызовы Simulation/Camera adapters находятся
непосредственно в `youbot_web.c`. После последних extraction-коммитов часть этих
вызовов закономерно переместилась ещё на один уровень вниз. Сначала нужно
обновить архитектурные тесты так, чтобы они проверяли границы и отсутствие
Webots/domain logic, а не конкретный промежуточный call site.

Метрики воспроизводятся через `(Get-Content <file>).Count`, `Select-String` для
`^#include`, `^#define` и mutable `^static`, а file inventory — через
`Get-ChildItem` с раздельными фильтрами production `*.c`, `*_test.c` и `*.h`.

## 2. Какие монолиты действительно нужно разбивать

| Приоритет | Кандидат | Доказательство | Решение |
|---|---|---|---|
| P0 | `webots/controllers/youbot_web/youbot_web.c` | 2471 строка, 85 includes, 210 defines, 48 mutable static declarations, крупный `run_navigation_step` примерно на 475 строк | Разбивать сейчас по runtime-boundaries |
| P1 | `src/features/planner/DashboardPlannerController.jsx` | 211 строк, около 20 локальных state-групп, много hook wiring, два огромных prop-object | После Webots: перейти к feature state/reducer и узким view models; не дробить JSX механически |
| P1 | `native/src/common.cpp` + `native/include/tsp/common.h` | 513 + 128 строк и один общий API для RNG, tour model, mutations, local search, population selection, random keys | Разбить на 5–6 алгоритмических библиотек с сохранением facade header на время миграции |
| P2 | `bridge/protocol/validation.cjs` | В одном файле algorithm defaults, route/zone/surface validation; defaults дублируют frontend registry | Разделить по контрактам после стабилизации shared schemas |
| P2 | frontend bundle | 727.68 KB; `xlsx` и Dashboard входят в основной chunk | Lazy route Dashboard и dynamic import `xlsx`; это bundle-монолит, а не source-file монолит |

Не следует разбивать только из-за размера:

- `controller_survey_coverage_intervals.c` и `controller_survey_integration.c`
  крупные, но обладают одной алгоритмической ответственностью;
- `routeAlgorithmRegistry.js` в основном декларативный registry;
- `dashboardTelemetry.js` остаётся единым клиентским normalization boundary;
- bridge servers/services уже имеют понятные transport/service границы.

## 3. Главный диагноз `youbot_web.c`

Уже выполнена значительная работа: файл сокращён примерно с 5804 до 2471 строки,
созданы отдельные modules для simulation, camera, mapping, navigation,
avoidance, survey, telemetry и runtime. Проблема теперь не в отсутствии helper
files, а в отсутствии нескольких владеющих состоянием runtime-компонентов.

Текущие оставшиеся обязанности `youbot_web.c`:

1. хранение и aliasing состояния через globals/macros;
2. сборка всех navigation/avoidance configs из десятков constants;
3. камера: capture, virtual render, obstacle hint, frame publication;
4. LiDAR: context, trace, persistent map merge;
5. mapping survey: callbacks, grid/contour/coverage route assembly, escape/scan;
6. reload route/zones/motion/runtime commands;
7. применение Robot Control decisions к status/error/motors;
8. telemetry snapshot assembly и запись;
9. startup, scheduling и shutdown Webots.

Антипаттерн, который нельзя продолжать: создавать по отдельному `.c/.h` для
каждой 3–10-строчной функции. В каталоге 247 файлов C/H: 92 production `.c`,
69 test `.c` и 86 headers. Следующие
единицы должны быть крупнее и владеть состоянием целого pipeline.

## 4. Целевая структура

```text
youbot_web.c                         150–300 строк: init -> step -> shutdown
controller_webots_application.*     composition/lifecycle, Webots-only effects
controller_control_config.*         typed immutable config groups
controller_perception_runtime.*     camera + LiDAR frame/state pipeline
controller_mapping_runtime.*        maps, survey generation, scan/escape requests
controller_command_runtime.*        file reloads and validated commands
controller_robot_control_step.*     SensorFrame -> StepResult/MotionCommand
controller_telemetry_publisher.*     runtime snapshot -> atomic state file
controller_sources.txt              единый manifest production/test sources
```

Разрешённое направление зависимостей:

| Модуль | Может зависеть от | Запрещено |
|---|---|---|
| pure domain (`navigation`, `avoidance`, `survey`) | `controller_types`, pure geometry/config | `wb_*`, filesystem, application/adapters |
| `controller_robot_control_step` | pure domain, `ControllerRuntime`, typed frames/config | Webots, filesystem, telemetry formatting |
| stateful runtimes (`perception`, `mapping`, `command`) | pure modules и собственные typed ports | прямые зависимости друг на друга, `wb_*` |
| file/Webots adapters | Webots SDK/filesystem + один узкий runtime port | domain decisions и владение Robot Control state |
| `controller_webots_application` | runtimes, adapters, scheduler | алгоритмы, parsing/formatting, mutable singleton globals |
| `youbot_web.c` | только application API и Webots loop | domain modules и runtime internals |

Размерные бюджеты: `controller_webots_application.c` <= 350 строк; каждый
stateful runtime `.c` <= 300 строк; `controller_robot_control_step.c` <= 350
строк и каждая функция в нём <= 120 строк; `youbot_web.c` <= 300 строк. Если
бюджет превышен, сначала пересматривается ответственность, а не создаются
passthrough modules.

Ключевые контракты:

```c
typedef struct {
  ControllerPose pose;
  ControllerPerceptionFrame perception;
  double simulation_time;
  int step;
} ControllerSensorFrame;

typedef enum {
  CONTROLLER_EFFECT_STOP = 1 << 0,
  CONTROLLER_EFFECT_APPLY_MOTION = 1 << 1,
  CONTROLLER_EFFECT_REQUEST_SURVEY_SCAN = 1 << 2,
  CONTROLLER_EFFECT_REQUEST_ORBIT_ESCAPE = 1 << 3,
  CONTROLLER_EFFECT_ROUTE_COMPLETED = 1 << 4
} ControllerStepEffectFlag;

typedef struct {
  unsigned effects;
  ControllerMotionCommand motion;
  char status[CONTROLLER_STATUS_CAPACITY];
  char error[CONTROLLER_ERROR_CAPACITY];
  double distance_to_target;
  unsigned continuation_token;
} ControllerStepResult;

typedef struct {
  unsigned continuation_token;
  ControllerStepEffectFlag completed_effect;
  int succeeded;
  ControllerSurveyRouteResult survey_route;
} ControllerEffectResult;
```

`controller_robot_control_step` не вызывает `wb_*`, не читает файлы и не пишет
telemetry. `controller_webots_application` применяет effect и владеет adapters.
Mapping survey отвечает на explicit request и возвращает validated route/result.
Status/error принадлежат значению результата, а не временным pointers. Если
Supervisor effect влияет на дальнейшее решение, application возвращает
`ControllerEffectResult` в `controller_robot_control_resume`; скрытого callback
из domain в Webots нет.

## Инварианты выполнения каждого task

- Любой новый/удалённый production `.c` одновременно обновляет
  `controller_sources.txt`; manifest parity test обязателен.
- Сначала focused failing test, затем минимальная реализация; red midpoint не
  коммитится и не передаётся следующему исполнителю.
- После Task 2 и каждого последующего task: focused test, `npm run test:webots`,
  `npm run webots:build`, manifest parity.
- После каждого chunk: `npm test`, `npm run lint`, `npm run build` и
  `npm run test:bridge` для затронутого transport flow.
- Architecture tests проверяют dependency direction, forbidden APIs, globals и
  budgets каждого нового runtime/application file.

## Chunk 1: Восстановить надёжный baseline

### Task 1: Исправить архитектурные тесты после последних extraction-коммитов

**Files:**
- Modify: `webots/controllerBuild.test.js`
- Inspect: `webots/controllers/youbot_web/controller_webots_zone_sync.c`
- Inspect: `webots/controllers/youbot_web/controller_webots_camera_range.c`
- Inspect: `webots/controllers/youbot_web/controller_webots_camera_perception.c`

- [ ] Переписать 6 failing assertions: проверять вызов в актуальном adapter
  boundary и отсутствие low-level реализации в coordinator.
- [ ] Запустить `npx vitest run webots/controllerBuild.test.js`.
- [ ] Запустить `npm test`; ожидается 70/70 files, 177/177 tests.
- [ ] Коммит: `test: align Webots architecture assertions with adapters`.

### Task 2: Ввести единый manifest исходников

**Files:**
- Create: `webots/controllers/youbot_web/controller_sources.txt`
- Modify: `webots/controllers/youbot_web/Makefile`
- Modify: `webots/controllers/youbot_web/build_youbot_web.bat`
- Modify: `webots/controllers/youbot_web/run_controller_tests.bat`
- Modify: `webots/controllerBuild.test.js`

- [ ] Сначала добавить failing test, требующий один canonical production source list.
- [ ] Научить три build path читать/генерировать список из manifest либо проверять
  manifest отдельным script без ручного тройного дублирования.
- [ ] Проверить single-test filter и полный `npm run test:webots`.
- [ ] Проверить `npm run webots:build`.
- [ ] Коммит: `build: centralize Webots controller source manifest`.

## Chunk 2: Убрать глобальное состояние и config explosion

### Task 3: Типизированные конфигурации

**Files:**
- Create: `webots/controllers/youbot_web/controller_control_config.h`
- Create: `webots/controllers/youbot_web/controller_control_config.c`
- Create: `webots/controllers/youbot_web/controller_control_config_test.c`
- Modify: `webots/controllers/youbot_web/youbot_web.c`
- Modify: `webots/controllers/youbot_web/controller_sources.txt`

- [ ] Создать группы `ControllerDriveConfig`, `ControllerNavigationConfig`,
  `ControllerPerceptionConfig`, `ControllerMappingConfig`,
  `ControllerScheduleConfig`.
- [ ] Characterization tests фиксируют текущие tolerances, cooldowns и speed limits.
- [ ] Заменить локальную сборку больших config literals на `config.navigation`,
  `config.perception`, `config.mapping`.
- [ ] Не менять числовые значения и порядок применения.
- [ ] Коммит: `refactor: centralize robot control configuration`.

## Chunk 3: Выделить stateful pipelines

### Task 4: Perception runtime

**Files:**
- Create: `controller_perception_runtime.h`
- Create: `controller_perception_runtime.c`
- Create: `controller_perception_runtime_test.c`
- Modify: `youbot_web.c` sections `init_sensors`, camera functions,
  `compute_lidar_obstacle_context`, `capture_lidar_trace`
- Modify: `controller_sources.txt`

- [ ] Characterization tests: LiDAR unavailable, front obstacle, expected wall,
  real camera, virtual camera, frame interval.
- [ ] Runtime owns camera/LiDAR metadata, trace and latest normalized perception frame.
- [ ] Webots reads remain behind `controller_webots_sensors` callbacks.
- [ ] Public API: `init`, `capture`, `frame`, `camera_publication_request`.
- [ ] BMP/JPEG encoding, scheduling и atomic write остаются в
  `controller_webots_camera_adapter`; runtime возвращает только typed request.
- [ ] После интеграции coordinator не содержит camera/LiDAR scalar globals.
- [ ] Коммит: `refactor: extract perception runtime pipeline`.

### Task 5: Mapping survey runtime

**Files:**
- Create: `controller_mapping_runtime.h`
- Create: `controller_mapping_runtime.c`
- Create: `controller_mapping_runtime_test.c`
- Modify: `youbot_web.c` mapping/survey block (примерно текущие строки 873–1785)
- Modify: `controller_sources.txt`

- [ ] Сценарные tests: default survey, contour/grid route, unsafe start,
  obstacle scan insertion, loop escape, map clear/write.
- [ ] Runtime owns `ControllerMappingStore`, survey callbacks/context and route
  generation config.
- [ ] Переиспользовать существующие geometry/grid/coverage modules; не создавать
  новые passthrough adapters.
- [ ] Public API возвращает explicit result/status, а file/Webots effects выполняет caller.
- [ ] Коммит: `refactor: extract mapping survey runtime`.

### Task 6: Command/reload runtime

**Files:**
- Create: `controller_command_runtime.h`
- Create: `controller_command_runtime.c`
- Create: `controller_command_runtime_test.c`
- Modify: `youbot_web.c` reload functions
- Modify: `controller_sources.txt`
- Reuse: `controller_route_zone_service.*`, `controller_runtime_command.*`,
  `controller_webots_motion_state.*`

- [ ] Тестировать mtime unchanged/changed, invalid file, duplicate command ID,
  route reload и zone sync request.
- [ ] Runtime возвращает effects (`ROUTE_CHANGED`, `ZONES_CHANGED`,
  `SPAWN_OBSTACLE`, `START_SURVEY`, `PROFILE_CHANGED`).
- [ ] File polling/parsing выполняют `controller_route_zone_service`,
  `controller_runtime_command` и motion-profile adapter; runtime принимает
  validated values/results и обновляет state/produces effects.
- [ ] Webots simulation sync остаётся в application adapter.
- [ ] Коммит: `refactor: extract controller command runtime`.

### Task 7: Контекст приложения без macro aliases

**Files:**
- Create: `controller_webots_application.h`
- Create: `controller_webots_application.c`
- Create: `controller_webots_application_test.c`
- Modify: `youbot_web.c`
- Modify: `controller_step.h`
- Modify: `controller_step.c`
- Modify: `controller_sources.txt`

- [ ] Теперь, когда runtime types существуют, ввести stack-owned
  `ControllerWebotsApplication` с paths, adapters, Robot Control runtime,
  mapping/perception/command runtimes, registries, reload metadata и step count.
- [ ] Передавать `ControllerWebotsApplication *` через typed callbacks; не
  создавать mutable singleton и не скрывать context в file-scope global.
- [ ] Переносить globals группами, не одной массовой заменой.
- [ ] Удалить aliases `persistent_map`, `navigation_status`,
  `configured_cruise_speed_mps` и path macros после миграции consumers.
- [ ] Architecture test запрещает `wb_*`/file parsing/domain branching в
  application и любые mutable file-scope globals в `youbot_web.c`.
- [ ] Budget `controller_webots_application.c` <= 350 строк.
- [ ] Коммит: `refactor: introduce Webots application context`.

## Chunk 4: Завершить Robot Control boundary

### Task 8: Characterize and implement the complete navigation step

**Files:**
- Create: `controller_robot_control_step.h`
- Create: `controller_robot_control_step_test.c`
- Create: `controller_robot_control_step.c`
- Modify: `controller_runtime.h`
- Modify: `controller_runtime.c`
- Modify: `youbot_web.c` function `run_navigation_step`
- Modify: `controller_sources.txt`
- Reuse: `controller_runtime.*`, `controller_navigation_service.*`,
  `controller_avoidance_service.*`

- [ ] Зафиксировать scenarios: no route, finished, relocation, waypoint advance,
  blocked zone, normal tracking, final alignment, avoidance start/continue,
  survey scan request, rejoin, orbit escape, free-space recovery.
- [ ] Для каждого сценария проверять effect, motion, status/error и state delta.
- [ ] Сначала focused tests должны падать из-за отсутствующего unified operation;
  не коммитить и не завершать task в красном состоянии.
- [ ] Скомпоновать существующие navigation/perception/avoidance modules без
  копирования алгоритмов.
- [ ] Вывести survey/orbit действия как explicit effect flags; если результат
  Supervisor action нужен для продолжения, обработать его отдельным
  `controller_robot_control_resume(runtime, effect_result)`.
- [ ] Оставить в `run_navigation_step`: capture frame, one service call, apply
  requested integration effect, apply motor command, publish status.
- [ ] Budget: `run_navigation_step` <= 120 строк; никаких `wb_*` внутри domain service.
- [ ] Коммит: `refactor: complete robot control step boundary`.

## Chunk 5: Тонкий Webots application shell

### Task 9: Telemetry publisher

**Files:**
- Create: `controller_telemetry_publisher.h`
- Create: `controller_telemetry_publisher.c`
- Create: `controller_telemetry_publisher_test.c`
- Modify: `youbot_web.c` function `write_state_snapshot`
- Modify: `controller_sources.txt`

- [ ] Перенести snapshot assembly из globals в typed input.
- [ ] Проверить navigation, motion, lidar, camera, maps и route fields.
- [ ] Pure assembler возвращает snapshot; atomic file write выполняет отдельный
  telemetry file adapter.
- [ ] Budget: callback в coordinator <= 15 строк.
- [ ] Коммит: `refactor: extract Webots telemetry publisher`.

### Task 10: Lifecycle application API

**Files:**
- Modify: `controller_webots_application.h`
- Modify: `controller_webots_application.c`
- Modify: `youbot_web.c`
- Modify: `controller_step.*`
- Modify: `controller_sources.txt`

- [ ] Реализовать `controller_webots_application_init`, `step`, `shutdown`.
- [ ] Свести `main` к Webots init/loop/cleanup и обработке init error.
- [ ] Устранить дублирующий `maybe_write_map()` при shutdown.
- [ ] Добавить architecture budgets: whole `youbot_web.c` <= 300 строк,
  includes <= 12, no domain constants, no file parsing, no JSON/BMP formatting.
- [ ] Коммит: `refactor: reduce youBot Web to application entrypoint`.

## Chunk 6: Проверка и только затем process extraction

### Task 11: Полный regression gate

- [ ] Создать deterministic fixtures:
  `webots/fixtures/smoke/route.csv`, `limit_zones.txt`,
  `surface_zones.txt`, `runtime_command.txt`.
- [ ] `npm test` — все tests PASS.
- [ ] `npm run lint` — PASS.
- [ ] `npm run build` — PASS.
- [ ] `npm run test:bridge` — PASS.
- [ ] `npm run test:webots` — 69+ tests PASS.
- [ ] `npm run webots:build` — PASS.
- [ ] Ручной Webots smoke по fixture route: зафиксировать status sequence
  `route_reloaded -> turning_to_path -> tracking_path -> finished`.
- [ ] Подать fixture zone/obstacle: увидеть `blocked_by_dynamic_zone` либо
  `avoiding_*`, затем восстановление `tracking_path`.
- [ ] Запустить mapping survey command: увидеть generation/start/resume status и
  обновление obstacle-map files.
- [ ] Проверить появление `camera_frame.bmp|jpg`, рост frame sequence и camera
  telemetry в Dashboard.
- [ ] Испортить обновление route fixture на контрольном шаге: ожидать явный
  `Route file is empty`/`Cannot open route.csv`, сохранение последнего валидного
  active route и отсутствие частично применённого route state. Изменение этого
  текущего поведения на STOP требует отдельного safety spec; heartbeat safe-stop
  отдельного Robot Control процесса относится к следующему плану.
- [ ] Сохранить `robot_state.json`, screenshot Dashboard и Webots log в
  `logs/smoke/<timestamp>/` как evidence; не коммитить runtime artifacts.
- [ ] Зафиксировать line/include/global budgets в `docs/ARCHITECTURE_INVENTORY.md`.

Только после этого вводить отдельный Robot Control process. Первым transport
контрактом должны стать versioned `SensorFrame` и `MotionCommand`, heartbeat и
safe-stop. Docker/Gateway не входят в этот план: они не должны скрывать
неустойчивую внутрипроцессную границу.

## 5. Follow-up backlog остальных монолитов (не исполняется этим планом)

Ниже только границы следующих самостоятельных specs/plans. Эти работы не должны
выполняться вместе с декомпозицией Webots и требуют собственного red-green плана.

### Dashboard planner

1. Объединить связанные `useState` в feature reducers: route, zones, energy,
   selection/presentation.
2. Передавать Workspace четыре узких view-model/action объекта вместо двух
   гигантских prop bags.
3. Оставить `DashboardPlannerController` composition root до 120–150 строк.
4. Добавить integration tests для optimize/send/survey, а не snapshot структуры props.

### Native common toolbox

1. `random.{h,cpp}` — `SeededRng`, Gaussian/Levy.
2. `tour.{h,cpp}` — Problem/Tour evaluation.
3. `operators.{h,cpp}` — crossover, mutation, neighbor/kick.
4. `local_search.{h,cpp}` — 2-opt and candidate improvement.
5. `population.{h,cpp}` — initial population, tournament/ref-set.
6. `random_keys.{h,cpp}` — keys conversion/evaluation.
7. Сохранить `common.h` как compatibility facade на одну миграционную фазу.

### Validation and bundle

1. Перенести algorithm schemas/defaults в `shared/contracts` и импортировать их
   в frontend/bridge вместо дублирования.
2. Разделить validators по `planning`, `route`, `zones`, `runtime-command`.
3. Lazy-load `/dashboard`; загружать `xlsx` только при импорте Excel.
4. Установить bundle budget и проверить, что initial chunk < 500 KB.

## 6. Критерии завершения

- `youbot_web.c` — entrypoint 150–300 строк, а не владелец алгоритмов/state.
- Один application context вместо множества globals и macro aliases.
- Robot Control можно тестировать без Webots SDK и файловой системы.
- Webots Adapter владеет только sensor/effect integration.
- Mapping, perception, command и telemetry pipelines имеют явные state owners.
- Build/test source lists не дублируются вручную.
- Все JS/C/build checks зелёные, ручной Webots smoke пройден.
- Процессная/контейнерная миграция начинается только после этой точки.
