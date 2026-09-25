// Dev only: stands in for `@footron/controls-client` so the real phone UI in
// controls/lib can be bundled and driven locally. useMessaging() gives the
// same { sendMessage } the real hook does, over a same-origin BroadcastChannel
// that dev/fake-messaging.js listens on in the wall page.
//
// It copies two behaviours of the real hook on purpose, because a UI that is
// fine without them can break with them (the "Open the editor" pad closed itself
// this way once):
//   * the listener is (re)registered whenever the callback's IDENTITY changes,
//     so an inline callback is re-registered on every render;
//   * every registration replays the recent message history, oldest first.
import { useEffect } from "react";

const QUEUE_SIZE = 10;
const received = [];
const live = new Set();
const channel = typeof BroadcastChannel === "function" ? new BroadcastChannel("loadpath-dev") : null;
if (channel) {
  channel.addEventListener("message", (event) => {
    received.push(event.data);
    while (received.length > QUEUE_SIZE) received.shift();
    live.forEach((listener) => listener(event.data));
  });
}

export function useMessaging(onMessage) {
  useEffect(() => {
    if (!onMessage) return undefined;
    live.add(onMessage);
    received.slice().forEach(onMessage);
    return () => {
      live.delete(onMessage);
    };
  }, [onMessage]);
  return { sendMessage: (body) => channel && channel.postMessage(body) };
}
