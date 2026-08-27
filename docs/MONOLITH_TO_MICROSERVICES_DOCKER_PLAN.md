# План полной декомпозиции и контейнеризации GPO

> **Для выполнения:** задачи исполняются последовательно в текущей сессии без
> субагентов. Каждый срез завершается тестами и отдельным коммитом.

**Цель:** разбить все реальные монолиты проекта, завершить микросервисную
архитектуру с сервисным слоем и подготовить независимые Docker-контейнеры,
включая Webots с доступным через браузер GUI.

**Архитектура:** frontend работает только через Gateway. Route, Planning,
Telemetry и Robot Control запускаются независимо и общаются через versioned
contracts. Webots Adapter содержит только Webots API и безопасно применяет
MotionCommand; алгоритмы управления принадлежат Robot Control.

**Стек:** React/Vite, Node.js, WebSocket/HTTP, C11, C++/CMake, Webots, Vitest,
Docker Compose, Xvfb/noVNC для Webots GUI.

---

## Целевая схема

```text
Browser
  -> Gateway
       -> Route Service
       -> Planning Service -> native solver
       -> Telemetry Service

Webots Container
  -> Webots Adapter -> SensorFrame -> Robot Control Service
  <- Webots Adapter <- MotionCommand <- Robot Control Service

Robot Control Service -> RobotState/TelemetryEvent -> Telemetry Service
```

## Фаза 0. Восстановить надёжную базовую линию

### Задача 0.1. Починить C test runner

**Файлы:**

- `webots/controllers/youbot_web/run_controller_tests.bat`
- `webots/controllerBuild.test.js`
- `webots/controllers/youbot_web/controller_webots_adapter_test.c`

- [ ] Добавить failing architecture test, проверяющий каждый production `.c` во всех трёх build lists.
- [ ] Зарегистрировать `controller_webots_simulation.c`.
- [ ] Передать Webots include path для adapter tests либо разделить pure и Webots-dependent headers.
- [ ] Выполнить `npm run test:webots` до итоговой строки `tests passed`.
- [ ] Выполнить `npm run webots:build`.
- [ ] Коммит: `fix: restore complete Webots controller tests`.

### Задача 0.2. Защитить telemetry transport

**Файлы:**

- `bridge/servers/telemetry-server.cjs`
- `bridge/servers/telemetry-server.test.mjs`

- [ ] Написать тест некорректного JSON и неподдерживаемого contractVersion.
- [ ] Убедиться, что тест падает на текущем обработчике.
- [ ] Добавить safe parsing и ServiceError без остановки сервера.
- [ ] Проверить, что следующее корректное сообщение доставляется.
- [ ] Коммит: `fix: reject invalid telemetry messages safely`.

## Фаза 1. Завершить декомпозицию `youbot_web.c`

Цель: 500–800 строк coordinator-кода без доменных алгоритмов и построения
Webots node strings.

### Задача 1.1. Simulation Adapter

**Файлы:**

- `controller_webots_simulation.{h,c}`
- `controller_webots_simulation_test.c`
- `youbot_web.c`

- [ ] Вынести registry dynamic nodes.
- [ ] Вынести создание/удаление limit-zone walls.
- [ ] Вынести surface-zone rendering.
- [ ] Вынести runtime obstacle spawn/eviction.
- [ ] Оставить чтение RuntimeCommand в service/infrastructure boundary.
- [ ] Проверить точные DEF names, размеры, цвета и bounding objects.
- [ ] Коммит: `refactor: extract Webots simulation adapter`.

### Задача 1.2. Camera Adapter

**Файлы:**

- создать `controller_webots_camera_adapter.{h,c}`
- создать `controller_webots_camera_adapter_test.c`
- изменить `youbot_web.c`

- [ ] Вынести получение реального кадра.
- [ ] Вынести virtual camera composition.
- [ ] Вынести JPEG/BMP temp replacement.
- [ ] Возвращать CameraObservation и frame metadata.
- [ ] Не переносить obstacle decision logic в adapter.
- [ ] Коммит: `refactor: extract Webots camera adapter`.

### Задача 1.3. Mapping persistence adapter

**Файлы:**

- создать `controller_mapping_store.{h,c}`
- создать `controller_mapping_store_test.c`
- изменить `youbot_web.c`

- [ ] Инкапсулировать persistent obstacle map state.
- [ ] Инкапсулировать camera obstacle/free maps.
- [ ] Вынести dirty flags и write intervals.
- [ ] Сохранить совместимость JSON/CSV/PNG путей.
- [ ] Коммит: `refactor: extract mapping persistence adapter`.

### Задача 1.4. Robot Control runtime context

**Файлы:**

- создать `controller_runtime.{h,c}`
- создать `controller_runtime_test.c`
- изменить `controller_navigation_service.{h,c}`
- изменить `youbot_web.c`

- [ ] Собрать route, zones, survey, avoidance и navigation state в context.
- [ ] Удалить соответствующие file-level globals из `youbot_web.c`.
- [ ] Определить `ProcessSensorFrame` и `CalculateMotionCommand`.
- [ ] Оставить Webots effects через explicit output actions.
- [ ] Коммит: `refactor: introduce Robot Control runtime context`.

### Задача 1.5. Тонкий coordinator

- [ ] Оставить init, capture frame, service call, effects, telemetry publish, cleanup.
- [ ] Добавить architecture test на отсутствие доменных функций в `youbot_web.c`.
- [ ] Зафиксировать фактический размер 500–800 строк.
- [ ] Прогнать весь C-набор и native build.
- [ ] Коммит: `refactor: finish youbot web decomposition`.

## Фаза 2. Завершить shared contracts

### Задача 2.1. Полный каталог контрактов

**Файлы:** `shared/contracts/*`

- [ ] Разнести envelope и payload schemas по отдельным модулям.
- [ ] Добавить Route, RobotState, SensorFrame, MotionCommand и ServiceError.
- [ ] Добавить strict version/type/payload validation.
- [ ] Добавить correlationId, causationId и timestamps.
- [ ] Добавить contract compatibility tests.
- [ ] Коммит: `feat: complete shared service contracts`.

### Задача 2.2. Idempotency

- [ ] Route Service хранит requestId применённых команд.
- [ ] Runtime commands не исполняются повторно после reconnect/restart.
- [ ] Planning request остаётся безопасным для retry.
- [ ] Добавить bounded retention и restart tests.
- [ ] Коммит: `feat: add command idempotency`.

## Фаза 3. Настоящие независимые сервисы

### Задача 3.1. Route Service process

- [ ] Добавить health/readiness endpoint.
- [ ] Добавить graceful shutdown.
- [ ] Изолировать state directory и configuration schema.
- [ ] Добавить process integration test.

### Задача 3.2. Planning Service process

- [ ] Добавить readiness native solver.
- [ ] Ограничить concurrency и время выполнения.
- [ ] Добавить graceful shutdown и error contract.
- [ ] Добавить Linux native solver integration test.

### Задача 3.3. Telemetry Service process

- [ ] Добавить health/readiness и latest-state HTTP endpoint.
- [ ] Ограничить message size и частоту.
- [ ] Реализовать backpressure/latest-frame policy.
- [ ] Добавить graceful shutdown.

### Задача 3.4. Robot Control Service process

- [ ] Создать отдельный executable/process для controller runtime.
- [ ] Принять SensorFrame, вернуть MotionCommand.
- [ ] Определить control-loop deadline.
- [ ] Реализовать safe-stop contract и heartbeat.
- [ ] Добавить deterministic replay tests.

## Фаза 4. Gateway и frontend

### Задача 4.1. Gateway Service

- [ ] Создать единый HTTP/WebSocket entrypoint.
- [ ] Проксировать Route, Planning и Telemetry.
- [ ] Нормализовать ServiceError.
- [ ] Добавить service discovery через environment.
- [ ] Добавить aggregate health/readiness.
- [ ] Коммит: `feat: add application gateway`.

### Задача 4.2. Frontend только через Gateway

- [ ] Заменить три host/port configuration на один Gateway URL.
- [ ] Добавить reconnect/backoff и понятные статусы недоступности.
- [ ] Lazy-load Dashboard и XLSX.
- [ ] Проверить route send, planning и telemetry flows.
- [ ] Коммит: `refactor: route frontend through gateway`.

## Фаза 5. Локальная интеграция без Docker

### Задача 5.1. Process launcher

- [ ] Запускать каждый сервис отдельным child process.
- [ ] Ждать readiness вместо фиксированной задержки.
- [ ] Корректно завершать все процессы.
- [ ] Сохранять независимый локальный state каждого разработчика.

### Задача 5.2. Полный E2E с Webots

- [ ] Frontend отправляет маршрут через Gateway.
- [ ] Route Service сохраняет/активирует маршрут.
- [ ] Webots Adapter отправляет SensorFrame.
- [ ] Robot Control возвращает MotionCommand.
- [ ] Telemetry появляется в Dashboard.
- [ ] Проверить остановку при потере Robot Control.
- [ ] Проверить продолжение маршрута при потере Planning/Gateway.

## Фаза 6. Очистка архитектуры

- [ ] Удалить compatibility bridge после миграции frontend.
- [ ] Удалить дублирующие root scripts/configs.
- [ ] Удалить generated `.obj`, `.exe`, test JSON/CSV из source tree.
- [ ] Расширить `.gitignore` и `.dockerignore`.
- [ ] Нормализовать структуру `services/`, `apps/`, `shared/`, `webots/`.
- [ ] Обновить README и команды запуска.
- [ ] Коммит: `chore: clean service architecture`.

## Фаза 7. Финальная Docker-контейнеризация

### Задача 7.1. Отдельные service images

- [ ] `gateway.Dockerfile`.
- [ ] `route-service.Dockerfile`.
- [ ] `planning-service.Dockerfile` с native solver.
- [ ] `telemetry-service.Dockerfile`.
- [ ] `robot-control.Dockerfile`.
- [ ] `frontend.Dockerfile`.

### Задача 7.2. Webots image

- [ ] Зафиксировать совместимую версию Webots.
- [ ] Установить controller/world/assets.
- [ ] Настроить Xvfb и software rendering fallback.
- [ ] Добавить noVNC/websockify для GUI в браузере.
- [ ] Подключить Adapter к Robot Control и Telemetry по Docker network.
- [ ] Реализовать heartbeat и safe stop.
- [ ] Добавить health check процесса симуляции.

### Задача 7.3. Compose topology

- [ ] Создать отдельные networks для public gateway и internal services.
- [ ] Добавить volumes для world, state, maps и logs.
- [ ] Настроить healthcheck/depends_on conditions.
- [ ] Не публиковать внутренние service ports наружу.
- [ ] Добавить profiles: `headless`, `gui`, `mock`.

### Задача 7.4. Docker E2E

- [ ] Собрать все images без локальных зависимостей.
- [ ] Запустить Compose на чистой машине.
- [ ] Открыть frontend и Webots GUI в браузере.
- [ ] Выполнить маршрут и увидеть телеметрию.
- [ ] Перезапустить каждый сервис и проверить recovery.
- [ ] Зафиксировать команды и troubleshooting.

## Оценка оставшейся работы

| Блок | Итерации |
|---|---:|
| Завершение `youbot_web` | 4–6 |
| Контракты и idempotency | 2–3 |
| Независимые процессы и Gateway | 4–6 |
| Локальный E2E и recovery | 3–4 |
| Очистка | 1–2 |
| Docker и Webots GUI | 4–6 |
| **Всего** | **18–27** |

## Финальные критерии готовности

- Все реальные монолиты разделены по ответственности.
- Каждый сервис имеет transport, service, domain и infrastructure boundaries.
- Frontend знает только Gateway.
- Webots Adapter не содержит навигационных алгоритмов.
- Robot Control безопасно останавливает робота при потере связи.
- Все контракты версионированы и валидируются.
- Все процессы имеют health/readiness и graceful shutdown.
- Локальный и Docker E2E проходят на чистой машине.
- Webots работает внутри контейнера, GUI доступен через браузер.
