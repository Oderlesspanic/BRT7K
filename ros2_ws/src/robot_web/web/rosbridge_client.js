(() => {
  class Ros {
    constructor(options = {}) {
      this.url = options.url || `ws://${window.location.hostname}:9090`;
      this.socket = null;
      this.callbacks = {
        connection: [],
        error: [],
        close: [],
        message: [],
      };
      this.serviceCallbacks = {};
      this.connect(this.url);
    }

    connect(url) {
      this.url = url;
      this.socket = new WebSocket(url);

      this.socket.addEventListener("open", () => {
        this.emit("connection");
      });

      this.socket.addEventListener("error", (event) => {
        this.emit("error", event);
      });

      this.socket.addEventListener("close", (event) => {
        this.emit("close", event);
      });

      this.socket.addEventListener("message", (event) => {
        try {
          const message = JSON.parse(event.data);
          if (message.op === "service_response" && message.id) {
            const callback = this.serviceCallbacks[message.id];
            if (callback) {
              delete this.serviceCallbacks[message.id];
              callback(message.values, message.result);
            }
          }
          this.emit("message", message);
        } catch (error) {
          console.error("rosbridge message parse failed:", error);
        }
      });
    }

    on(eventName, callback) {
      if (!this.callbacks[eventName]) {
        this.callbacks[eventName] = [];
      }
      this.callbacks[eventName].push(callback);
    }

    emit(eventName, payload) {
      const callbacks = this.callbacks[eventName] || [];
      callbacks.forEach((callback) => callback(payload));
    }

    sendEncoded(message) {
      const send = () => this.socket.send(JSON.stringify(message));

      if (this.socket && this.socket.readyState === WebSocket.OPEN) {
        send();
        return;
      }

      if (this.socket && this.socket.readyState === WebSocket.CONNECTING) {
        this.socket.addEventListener("open", send, { once: true });
      }
    }
  }

  class Topic {
    constructor(options) {
      this.ros = options.ros;
      this.name = options.name;
      this.messageType = options.messageType;
      this.callbacks = [];
      this.advertised = false;

      this.ros.on("message", (message) => {
        if (message.op === "publish" && message.topic === this.name) {
          this.callbacks.forEach((callback) => callback(message.msg));
        }
      });
    }

    subscribe(callback) {
      this.callbacks.push(callback);
      this.ros.sendEncoded({
        op: "subscribe",
        topic: this.name,
        type: this.messageType,
      });
    }

    advertise() {
      if (this.advertised) return;

      this.ros.sendEncoded({
        op: "advertise",
        topic: this.name,
        type: this.messageType,
      });
      this.advertised = true;
    }

    publish(message) {
      const sendPublish = () => this.ros.sendEncoded({
        op: "publish",
        topic: this.name,
        msg: message,
      });

      if (!this.advertised) {
        this.advertise();
        setTimeout(sendPublish, 20);
        return;
      }

      sendPublish();
    }
  }

  class Service {
    constructor(options) {
      this.ros = options.ros;
      this.name = options.name;
      this.serviceType = options.serviceType;
    }

    callService(request = {}, callback = () => {}) {
      const id =
        `${this.name}:${Date.now()}:${Math.random().toString(16).slice(2)}`;

      this.ros.serviceCallbacks[id] = callback;
      this.ros.sendEncoded({
        op: "call_service",
        id,
        service: this.name,
        type: this.serviceType,
        args: request,
      });
    }
  }

  class Message {
    constructor(values = {}) {
      Object.assign(this, values);
    }
  }

  window.ROSLIB = {
    Ros,
    Topic,
    Service,
    Message,
  };
})();
