#!/usr/bin/env node
// putbin.js - Raw chunked uploader for Arduino USB CDC
// Usage: node putbin.js --port /dev/ttyACM0 --file ./big.bin --name big.bin [--baud 115200] [--chunk 16384]
const fs = require('fs');
const path = require('path');
const { SerialPort } = require('serialport');
const CRC32 = require('crc-32'); // npm i crc-32

function arg(k, d) {
  const i = process.argv.indexOf(k);
  return i > 0 ? process.argv[i + 1] : d;
}
const portPath = arg('--port');
const filePath = arg('--file');
const remoteName = arg('--name', path.basename(filePath || ''));
const baud = parseInt(arg('--baud', '115200'), 10);
const CHUNK = parseInt(arg('--chunk', '16384'), 10);

if (!portPath || !filePath || !remoteName) {
  console.error('Usage: node putbin.js --port <tty> --file <local.bin> --name <remote.bin> [--baud 115200] [--chunk 16384]');
  process.exit(1);
}
const stat = fs.statSync(filePath);
if (!stat.isFile()) {
  console.error('Not a regular file:', filePath);
  process.exit(1);
}
const total = stat.size;

// Frame format (little endian):
// [0..3]  = 0xA5 0x5A 0x4B 0x52 (magic = "A5 5A KR")
// [4..7]  = uint32 offset
// [8..11] = uint32 length
// [12..15]= uint32 crc32(payload)
// [16.. ] = payload
// Final commit frame: offset=0xFFFFFFFF, length=0
const MAGIC = Buffer.from([0xA5, 0x5A, 0x4B, 0x52]);

function makeFrame(offset, payload) {
  const len = payload ? payload.length : 0;
  const buf = Buffer.alloc(16 + len);
  MAGIC.copy(buf, 0);
  buf.writeUInt32LE(offset >>> 0, 4);
  buf.writeUInt32LE(len >>> 0, 8);
  const crc = len ? (CRC32.buf(payload) >>> 0) : 0;
  buf.writeUInt32LE(crc >>> 0, 12);
  if (len) payload.copy(buf, 16);
  return buf;
}

(async () => {
  const port = new SerialPort({ path: portPath, baudRate: baud, autoOpen: true });
  const reader = port.readable;
  const writer = port.writable;
  const enc = new TextEncoder();
  const dec = new TextDecoder();

  async function readLine(timeoutMs = 10000) {
    let acc = '';
    const start = Date.now();
    for (;;) {
      const chunk = await new Promise((resolve, reject) => {
        const to = setTimeout(() => resolve(null), 50);
        port.once('data', (d) => { clearTimeout(to); resolve(d); });
      });
      if (chunk) acc += chunk.toString('utf8');
      if (acc.indexOf('\n') >= 0) {
        const i = acc.indexOf('\n');
        const line = acc.slice(0, i).trim();
        // keep remainder in buffer (not strictly needed here)
        return line;
      }
      if (Date.now() - start > timeoutMs) throw new Error('Timeout waiting for line');
    }
  }

  function writeStr(s) {
    return new Promise((res, rej) => port.write(s, (e) => e ? rej(e) : res()));
  }

  console.log(`Opening ${portPath} @ ${baud} ...`);
  await new Promise((r) => setTimeout(r, 200));

  // Flush any pending
  port.flush(() => {});
  await new Promise((r) => setTimeout(r, 100));

  // Issue command to prepare slot
  const cmd = `putbin ${remoteName} ${total}\r\n`;
  await writeStr(cmd);
  console.log('Sent:', cmd.trim());

  // Wait for READY
  let line;
  do {
    line = await readLine(15000);
    console.log('<', line);
  } while (!/^READY($|\s)/i.test(line));
  console.log('Uploader: starting data stream...');

  // Stream file
  const fd = fs.openSync(filePath, 'r');
  let sent = 0;
  const startTime = Date.now();
  while (sent < total) {
    const toSend = Math.min(CHUNK, total - sent);
    const payload = Buffer.allocUnsafe(toSend);
    const rd = fs.readSync(fd, payload, 0, toSend, sent);
    if (rd !== toSend) throw new Error('Short read');
    const frame = makeFrame(sent, payload);
    await new Promise((res, rej) => port.write(frame, (e) => e ? rej(e) : res()));
    sent += toSend;

    if ((sent % (512 * 1024)) === 0 || sent === total) {
      const dt = (Date.now() - startTime) / 1000;
      const mb = (sent / (1024 * 1024)).toFixed(2);
      const spd = (sent / (1024 * 1024) / Math.max(dt, 0.001)).toFixed(2);
      console.log(`  ${mb} MiB sent  (${spd} MiB/s)`);
    }
  }
  fs.closeSync(fd);

  // Commit
  const commit = makeFrame(0xFFFFFFFF >>> 0, Buffer.alloc(0));
  await new Promise((res, rej) => port.write(commit, (e) => e ? rej(e) : res()));
  console.log('Commit sent, waiting for OK...');

  for (;;) {
    line = await readLine(20000);
    console.log('<', line);
    if (/^OK($|\s)/i.test(line)) break;
    if (/^ERR/i.test(line)) throw new Error('Device reported error');
  }

  console.log('Done.');
  process.exit(0);
})().catch((e) => {
  console.error('ERROR:', e.message || e);
  process.exit(1);
});