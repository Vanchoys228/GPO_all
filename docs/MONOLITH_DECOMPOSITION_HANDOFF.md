# Handoff: декомпозиция `youbot_web`

Дата: 2026-08-28

## Цель

Декомпозировать `webots/controllers/youbot_web/youbot_web.c` до тонкого Webots coordinator, затем перейти к сервисному слою, независимым процессам, Gateway и Docker. Подробный план: `docs/MONOLITH_TO_MICROSERVICES_DOCKER_PLAN.md`.

## Правило проверок

- После малого изменения: только целевой C/JS тест и при необходимости `npm run webots:build`.
- После законченного крупного блока: `npm test`, `npm run lint`, `npm run build`, `npm run test:webots`, `npm run webots:build`.
- Не запускать полный набор после каждой маленькой правки.

## Рабочая копия

- Основная рабочая ветка пользователя: `C:\GPO_nimmwee`.
- Изолированная worktree для этой работы: `C:\GPO_nimmwee\.worktrees\controller-test-runner`.
- Ветка worktree: `codex/controller-test-runner`.
- Запуск frontend/bridge был выполнен из worktree: UI `http://127.0.0.1:5173`, bridge — порты `9001` и `9002`.

## Завершено и закоммичено

- `126c39d` — восстановлен полный C test runner Webots.
- `cf15c1c` — безопасная валидация telemetry transport.
- `c5476ea`, `07b32f2`, `8913c48`, `4e80708`, `f92e366`, `fe7b3e8`, `410ecc6`, `91f68a0` — Simulation Adapter: dynamic Webots nodes, limit/surface zones и runtime obstacles вынесены из coordinator.
- `23c9654`, `9e350e1`, `ea74bdc`, `57a437e`, `414e1da`, `7fa546b`, `0b97df4` — camera fusion/virtual camera/renders вынесены в специализированные модули.
- `37b9f74` — `controller_webots_camera_adapter`: атомарная публикация BMP/JPEG и frame metadata.
- `0b03843` — `controller_mapping_store`: persistent/camera/free map state, clear/write persistence.

Размер `youbot_web.c` уменьшен с 3194 до 2866 строк после последнего завершённого Mapping Store коммита.

## Последние подтверждённые проверки

- Полный регресс после Simulation Adapter: `npm test` — 143/143; `npm run lint` — успешно; `npm run build` — успешно; `npm run test:webots` завершился с кодом 0; `npm run webots:build` — успешно.
- После `0b03843`: целевой `controller_mapping_store_test.c`, `webots/controllerBuild.test.js` (17 tests) и `npm run webots:build` — успешно.
- Пользователь вручную проверил работоспособность проекта после сборки и сообщил, что всё работает.

## Важное: текущая незавершённая работа

Не продолжать существующие заготовки Runtime/RouteZone как готовый код.

- Попытка интегрировать `ControllerRuntime` через C macro aliases была отменена: macros подменяли одноимённые поля структур в designated initializers и ломали build.
- Временные `controller_runtime.{h,c}` удалены.
- Временный `controller_route_zone_service_test.c` удалён.
- Проверить `git status --short`: могут остаться generated-файлы (`web_state/camera_frame.jpg`, `webots/worlds/.youbot_only.wbproj`) и форматные изменения. Не удалять пользовательские generated-файлы без необходимости.
- Последняя послеоткатная проверка: `npm run webots:build` успешно.

## Следующий правильный большой срез

### RouteZoneService

Создать полноценный `controller_route_zone_service.{h,c}` одним завершённым блоком, а не маленькими частями.

Ответственность:

1. reload route, limit zones и surface zones;
2. file mtime/change detection;
3. parsing и validation через существующие `controller_route`/`controller_zones` APIs;
4. вернуть explicit result: что изменилось, новые validated data и статус/error;
5. coordinator применяет результат: route lifecycle либо Webots simulation sync.

Не использовать macros для подмены state. Предпочитать explicit input/output structs и передачу указателей.

Проверки перед одним коммитом:

1. Новый C test service layer (red → green).
2. `npx vitest run webots/controllerBuild.test.js`.
3. `npm run webots:build`.
4. После завершения RouteZoneService как крупного блока — полный набор проверок по правилу выше.

## После RouteZoneService

Следующий крупный блок — `RobotControlRuntime`, но через явный `ControllerRuntime` и API, а не через preprocessor aliases. В context должны войти route/zones, navigation, avoidance и mapping-survey state; Webots effects остаются в coordinator.
