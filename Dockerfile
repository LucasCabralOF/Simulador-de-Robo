# Multi-stage build for DH Robot Kinematics Platform
# Stage 1: Build frontend bundle
FROM node:22-alpine AS builder
WORKDIR /app
COPY package*.json ./
RUN npm ci
COPY . .
RUN npm run build

# Stage 2: Production runtime
FROM node:22-alpine AS runner
WORKDIR /app
ENV NODE_ENV=production \
    PORT=3000 \
    HOST=0.0.0.0 \
    DB_PATH=/app/data/database.sqlite

# Install production dependencies only
COPY package*.json ./
RUN npm ci --omit=dev && npm cache clean --force

# Copy server and kinematics runtime
COPY server ./server
COPY server.js ./
COPY src/kinematics.js ./src/kinematics.js
COPY --from=builder /app/dist ./dist

# Persistent data directory
RUN mkdir -p /app/data && chown -R node:node /app

USER node
EXPOSE 3000

CMD ["node", "server.js"]
