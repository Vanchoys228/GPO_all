FROM node:22-bookworm-slim AS native-build

RUN apt-get update \
  && apt-get install --yes --no-install-recommends cmake g++ make \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY native ./native

RUN cmake -S native -B native/build -DCMAKE_BUILD_TYPE=Release \
  && cmake --build native/build --config Release --parallel

FROM node:22-bookworm-slim AS dependencies

WORKDIR /app

COPY package.json package-lock.json ./

RUN npm ci --omit=dev --ignore-scripts \
  && npm cache clean --force

FROM node:22-bookworm-slim AS runtime

ENV NODE_ENV=production \
  BRIDGE_BIND_HOST=0.0.0.0 \
  TELEMETRY_PORT=9001 \
  ROUTE_PORT=9002 \
  SOLVER_PORT=9003 \
  WEB_STATE_DIR=/state \
  SOLVER_PATH=/app/native/build/gpo_route_solver

WORKDIR /app

COPY --from=dependencies /app/node_modules ./node_modules
COPY --from=native-build /app/native/build/gpo_route_solver ./native/build/gpo_route_solver
COPY bridge ./bridge
COPY shared ./shared
COPY bridge-config.cjs package.json ws-bridge.cjs ./

RUN mkdir -p /state \
  && chown -R node:node /app /state

USER node

EXPOSE 9001 9002 9003

CMD ["node", "ws-bridge.cjs"]
