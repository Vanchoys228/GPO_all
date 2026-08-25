# Architecture Inventory

## Точки входа

| Контур | Точка входа | Связь с запуском | Решение |
|---|---|---|---|
| Frontend | `src/main.jsx` → `src/App.jsx` | `index.html`, Vite | оставить |
| Dashboard | `src/pages/Dashboard.jsx` | route `/dashboard` | оставить |
| Bridge | `ws-bridge.cjs` | `npm run bridge`, Docker, bridge smoke test | оставить |
| Mock telemetry | `telemetry-server.cjs` | `npm run telemetry:mock` | оставить как dev utility |
| Native solver | `native/apps/gpo_route_solver.cpp` | CMake/MSVC, вызывается bridge | оставить |
| Webots | `webots/controllers/youbot_web/youbot_web.c` | Webots world controller | оставить |
| .NET | удалённая legacy-заглушка | не участвовала в npm, Vite, bridge или Docker flow | удалено |

## Верхнеуровневые каталоги

| Каталог | Назначение | Состояние |
|---|---|---|
| `bridge/` | протокол, серверы, telemetry, solver adapter, state store | граница понятна |
| `docker/` | образ bridge/native solver | оставить до Docker-этапа |
| `docs/` | актуальный план и технические материалы | требуется свёртка временных plan/spec файлов |
| `native/` | исходники и сборка CLI solver | `native/build/` является генерируемым |
| `public/` | опциональные статические файлы Vite | после удаления шаблонных assets пустой каталог удалён; создать снова при появлении реальных assets |
| `scripts/` | smoke-проверки | оставить |
| `shared/` | межпроцессный координатный контракт | оставить |
| `src/` | React frontend | декомпозиция завершена |
| `web_state/` | runtime-обмен | хранить только `.gitkeep` |
| `webots/` | world и C controller | исходники оставить, `.exe` не коммитить |

## Проверенные кандидаты

| Кандидат | Категория | Доказательство | Предварительное решение |
|---|---|---|---|
| `Program.cs` | мёртвая заглушка | содержал только `Hello, World!`; не входил в Node/Vite/Docker flow | удалён |
| `bnts-frontend.csproj` | устаревшая конфигурация | собирал только заглушку .NET 7; npm является фактическим frontend toolchain | удалён |
| `bridge-config.cjs` | compatibility facade | импортируется mock telemetry и копируется Dockerfile | оставить |
| `telemetry-server.cjs` | dev utility | отдельная команда `npm run telemetry:mock` | оставить, явно описать в README |
| `native/build/**` | build artifact | исключён `.gitignore`; solver воспроизводимо собирается CMake/MSVC | удалён из рабочей копии |
| `webots/controllers/youbot_web/*.exe` | build/test artifacts | исключены `.gitignore`; рядом присутствуют исходные `.c` | 39 файлов удалены из рабочей копии |
| `web_state/*`, кроме `.gitkeep` | runtime state | исключено `.gitignore`; bridge и Webots создают файлы во время работы | не коммитить |
| `papaparse` | неиспользуемая npm-зависимость | не было импортов вне lock/package metadata | удалена из package и lockfile |
| `read-excel-file` | неиспользуемая npm-зависимость | импорт Excel выполняется через `xlsx`; других импортов не было | удалена из package и lockfile |
| `public/vite.svg` | шаблонный asset | был стандартным favicon Vite и не относился к GPO | удалён вместе со ссылкой из `index.html` |
| `src/App.css` | шаблонные стили | файл не импортировался; селекторы `.logo`, `.card` и `.read-the-docs` не использовались | удалён |
| `src/assets/react.svg` | шаблонный asset | ссылок в `src` и конфигурации не было | удалён |
| `src/components/dashboard/CollapsibleSection.jsx` | мёртвый UI-компонент | ни одного импорта или JSX-использования; sidebar уже разделены на постоянные секции | удалён |
| section-state в `plannerUiState.js` | мёртвое состояние | использовалось только удалённым `CollapsibleSection`; Dashboard использует только collapse панелей | удалено, panel-state оставлено |
| `public/map.png` | legacy asset | шахматный фон; ссылок в исходниках, CSS и Webots нет; UI рисует карту в canvas | удалён |
| `webots/worlds/.youbot_only.wbproj` | Webots metadata | расположен рядом с используемым `.wbt`; может хранить настройки проекта | оставить до запуска Webots GUI |
| `webots/worlds/.youbot_only.jpg` | Webots preview | прямых текстовых ссылок нет; служебный preview мира | оставить до запуска Webots GUI |
| `PROJECT_MODULARITY.drawio` | редактируемая архитектурная схема | обновлена под hooks/model/services, facade и bridge modules | оставить |
| `PROJECT_MODULARITY.png` | устаревший render | не имел входящих ссылок и не мог обновляться без отдельного renderer | удалить, source остаётся в `.drawio` |
| `docs/superpowers/**` | временные implementation plans/specs | 24 файла без входящих ссылок; реализованные решения описаны в актуальных docs и тестах | удалены |

## Npm-зависимости

- Подтверждённо используются: `dotenv`, `react`, `react-dom`,
  `react-router-dom`, `ws`, `xlsx`.
- Удалены как неиспользуемые: `papaparse`, `read-excel-file`.
- Dev-зависимости соответствуют Vite, ESLint, Tailwind/PostCSS и Vitest.

## Frontend imports и exports

- ESLint не обнаруживает неиспользуемых локальных imports.
- Единственным полностью неимпортируемым production-компонентом был
  `CollapsibleSection`; он удалён вместе с относящимися только к нему helpers.
- Экспорты `isNavigationOffRoute`, `parsePlannerTableRows` и
  `resolveSurfaceAtPoint` используются напрямую тестами и оставлены как
  тестируемые чистые функции.
- Константы `runtimeConfig.js` и `SURFACE_PROFILES` используются внутри своих
  модулей для построения публичных URL и моделей; их файлы не являются мёртвыми.

## C/C++ inventory

- Все 19 production `.c`-файлов Webots controller перечислены в
  `build_youbot_web.bat`.
- Для каждого controller-модуля есть соответствующий тестовый исходник, кроме
  композиционного `youbot_web.c`.
- Объявленные в заголовках функции имеют реализации и production/test-вызовы;
  доказанно мёртвых C-функций в этом проходе не найдено.
- Все реализации native solver из `native/src` входят в `native/CMakeLists.txt`,
  а четыре алгоритма вызываются CLI solver.

## Похожие, но не дублирующие модули

- Корневой `telemetry-server.cjs` генерирует mock telemetry, тогда как
  `bridge/servers/telemetry-server.cjs` обслуживает production WebSocket flow.
- `bridge/telemetry/normalizer.cjs` нормализует серверный file/Webots payload,
  а `src/lib/dashboardTelemetry.js` поддерживает клиентское накопительное UI
  состояние. Объединение создаст нежелательную связь frontend и CommonJS bridge.
- `bridge-config.cjs` сохраняется как совместимый facade над
  `bridge/config/runtime-config.cjs`.

## Следующий проход

1. Проверить чистую сборку native/Webots перед финальным Docker-этапом.

## Отложенные кандидаты

- `START_HERE_WINDOWS.md` частично повторяет README, но содержит Docker health
  check и ручной Webots checklist. Объединить с README после завершения Docker,
  затем удалить отдельный файл.
- `REFACTORING_PLAN.md` — исторический отчёт о выполненной декомпозиции. Его
  можно удалить после фиксации итоговой архитектуры в README и PR description,
  но сейчас он остаётся источником решений и результатов C/Webots-срезов.
- `.youbot_only.wbproj` и `.youbot_only.jpg` остаются до ручной проверки мира в
  Webots GUI: отсутствие текстовых ссылок недостаточно для безопасного удаления
  Webots metadata/preview.
