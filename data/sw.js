// Service worker for the HTTPS copy of the panel (GitHub Pages, BLE mode).
// index.html registers it only when the page is served over https:, never
// from the ESP (http://192.168.4.1). Network first for the page itself so a
// new panel version shows up at once; cache first for the static assets.
const CACHE = 'turbina-v1';
const ASSETS = ['./', 'index.html', 'manifest.json', 'icon.svg', 'icon-192.png', 'icon-512.png'];

self.addEventListener('install', (e) => {
  // One missing icon must not break the install, so add the files one by one.
  e.waitUntil(
    caches.open(CACHE)
      .then((c) => Promise.all(ASSETS.map((a) => c.add(a).catch(() => {}))))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener('activate', (e) => {
  e.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k))))
      .then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', (e) => {
  const req = e.request;
  if (req.method !== 'GET') return;
  const url = new URL(req.url);
  if (url.origin !== self.location.origin) return;
  const isPage = req.mode === 'navigate' || url.pathname.endsWith('/') || url.pathname.endsWith('/index.html');
  if (isPage) {
    e.respondWith(
      fetch(req)
        .then((res) => {
          if (res.ok) {
            const copy = res.clone();
            caches.open(CACHE).then((c) => c.put('index.html', copy));
          }
          return res;
        })
        .catch(() => caches.match('index.html').then((m) => m || caches.match('./')))
    );
    return;
  }
  e.respondWith(caches.match(req).then((m) => m || fetch(req)));
});
