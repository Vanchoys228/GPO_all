# Repository Guidelines

## Project Structure & Module Organization
This repository is a Vite + React frontend with optional WebSocket bridge utilities.

- `src/`: application code.
- `src/pages/`: page-level views (`Dashboard.jsx`, `RoutePlanner.jsx`).
- `src/components/`: reusable UI pieces (`Navbar.jsx`, `MapCanvas.jsx`).
- `src/lib/`: shared helpers (`telemetry.js`).
- `public/`: static runtime assets (`map.png`, `vite.svg`).
- Root configs: `vite.config.js`, `eslint.config.js`, `tailwind.config.js`, `postcss.config.js`.
- Runtime bridge scripts: `ws-bridge.cjs` and `telemetry-server.cjs`.

`Program.cs` and `bnts-frontend.csproj` exist, but current frontend development is driven by Node/Vite.

## Build, Test, and Development Commands
- `npm install`: install dependencies.
- `npm run dev`: start Vite dev server.
- `npm run bridge`: run WebSocket bridge (`ws://127.0.0.1:9001` and `:9002`).
- `npm run build`: produce production build in `dist/`.
- `npm run preview`: preview the production bundle locally.
- `npm run lint`: run ESLint checks.

Example local workflow: run `npm run bridge` in one terminal and `npm run dev` in another.

## Coding Style & Naming Conventions
- Use functional React components and ES modules.
- Match existing style: 2-space indentation, semicolons, double quotes.
- Component filenames use PascalCase (`MapPlaceholder.jsx`); utility files use camelCase (`telemetry.js`).
- Keep page-level state/flows in `src/pages`, reusable UI in `src/components`.
- Use `@` import alias for `src` paths where it improves readability.

## Testing Guidelines
No automated test runner is currently configured (`package.json` has no `test` script and no coverage gate).

Before opening a PR:
- Run `npm run lint`.
- Smoke-test key flows manually: map rendering, telemetry connection status, and route sending via `ws-bridge.cjs`.

If you add tests, prefer Vitest + React Testing Library and use `*.test.jsx` naming.

## Commit & Pull Request Guidelines
Git history is not available in this workspace snapshot, so commit conventions could not be inferred directly.

Use Conventional Commit style going forward:
- `feat: add route validation before send`
- `fix: reconnect route websocket on close`

PR checklist:
- Clear summary and scope.
- Linked task/issue.
- Screenshots or short video for UI changes.
- Verification notes with commands run (for example `npm run lint`).
