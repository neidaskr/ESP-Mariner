export class BridgeSocket {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this.listeners = new Set();
    this.stateListeners = new Set();
    this.retryMs = 1200;
  }

  connect() {
    this.ws = new WebSocket(this.url);

    this.ws.addEventListener("open", () => this.emitState(true));
    this.ws.addEventListener("close", () => {
      this.emitState(false);
      setTimeout(() => this.connect(), this.retryMs);
    });
    this.ws.addEventListener("error", () => {
      this.ws?.close();
    });

    this.ws.addEventListener("message", (ev) => {
      try {
        const data = JSON.parse(ev.data);
        for (const cb of this.listeners) cb(data);
      } catch {
        // Ignore malformed frame.
      }
    });
  }

  onFrame(cb) {
    this.listeners.add(cb);
    return () => this.listeners.delete(cb);
  }

  onState(cb) {
    this.stateListeners.add(cb);
    return () => this.stateListeners.delete(cb);
  }

  emitState(isUp) {
    for (const cb of this.stateListeners) cb(isUp);
  }

  send(obj) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false;
    this.ws.send(JSON.stringify(obj));
    return true;
  }
}
