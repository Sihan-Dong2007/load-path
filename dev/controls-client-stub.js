// Dev only: stands in for `@footron/controls-client` so the real phone UI in
// footron/controls/lib can be bundled and driven locally. useMessaging() gives the
// same { sendMessage } the real hook does, but over a same-origin BroadcastChannel
// that dev/fake-messaging.js listens on in the wall page.
const channel = typeof BroadcastChannel === "function" ? new BroadcastChannel("loadpath-dev") : null;
export function useMessaging() {
  return { sendMessage: (body) => channel && channel.postMessage(body) };
}
