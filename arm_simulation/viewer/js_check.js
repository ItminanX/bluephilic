// Usage: node js_check.js vectors.json   - compares the viewer's JavaScript maths with numbers computed by the Python code.
const fs = require('fs');
const html = fs.readFileSync(__dirname + '/index.html', 'utf8');
const code = html.slice(html.indexOf('<script>') + 8, html.lastIndexOf('</script>'));
const mod = { exports: {} };
new Function('module', code)(mod);
const J = mod.exports, V = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
let worst = 0, n = 0, bad = [];
const cmp = (name, a, b, tol) => { const d = Math.max(...a.map((v, i) => Math.abs(v - b[i]))); worst = Math.max(worst, d); n++; if (!(d < tol)) bad.push(name + ' diff ' + d); };
for (const c of V.fk) {
  const f = J.fk(c.q, V.P);
  cmp('m3', f.m3, c.m3, 1e-9); cmp('s1', f.s1, c.s1, 1e-9); cmp('tcp', f.tcp, c.tcp, 1e-9); cmp('cam', f.cam, c.cam, 1e-9);
  cmp('right', f.right, c.right, 1e-9); cmp('up', f.up, c.up, 1e-9); cmp('fwd', f.fwd, c.fwd, 1e-9);
  if (J.collides(c.q, V.P) !== c.collides) bad.push('collides mismatch at ' + c.q);
}
for (const c of V.ik) {
  const got = J.ikAll(c.t, V.P), want = c.sols;
  if (got.length !== want.length) { bad.push('ik count ' + got.length + ' vs ' + want.length + ' at ' + c.t); continue; }
  got.sort((a, b) => a[1] - b[1]); want.sort((a, b) => a[1] - b[1]);
  got.forEach((q, i) => cmp('ik', q, want[i], 1e-7));
  const best = J.ik(c.t, V.P, c.qref); if (want.length) cmp('ikbest', best, c.best, 1e-7);
}
console.log(bad.length ? 'FAIL ' + bad.slice(0, 5).join(' | ') : 'OK compared ' + n + ' values, worst difference ' + worst.toExponential(2));
process.exit(bad.length ? 1 : 0);
