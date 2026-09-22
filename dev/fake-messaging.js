// Dev only. Stands in for the vendored @footron/messaging client so the wall page
// can be driven by dev/phone.html on this machine, without a Footron messaging
// server: both sides talk over a same-origin BroadcastChannel instead of a
// WebSocket. It implements just the part of the real client's API the wall
// uses (addMessageListener / removeMessageListener / mount / unmount / sendMessage).
// serve.py --dev injects it after the real client, replacing FootronMessaging.
window.FootronMessaging = {
  Messaging: class {
    constructor() {
      this.channel = new BroadcastChannel("loadpath-dev");
      this.listeners = new Set();
      this.channel.onmessage = (event) => this.listeners.forEach((listener) => listener(event.data));
    }
    addMessageListener(listener) { this.listeners.add(listener); }
    removeMessageListener(listener) { this.listeners.delete(listener); }
    mount() {}
    unmount() { this.channel.close(); }
    sendMessage(body) { this.channel.postMessage(body); }
  },
};
