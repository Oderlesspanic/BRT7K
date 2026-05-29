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
          this.emit("message", JSON.parse(event.data));
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

    publish(message) {
      this.ros.sendEncoded({
        op: "publish",
        topic: this.name,
        msg: message,
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
    Message,
  };
})();
