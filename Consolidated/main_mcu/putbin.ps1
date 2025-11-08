param(
  [Parameter(Mandatory=$true)] [string]$Port,
  [Parameter(Mandatory=$true)] [string]$File,
  [Parameter(Mandatory=$true)] [string]$Name,
  [int]$Baud = 115200,
  [int]$Chunk = 16384,
  [int]$LineTimeoutMs = 15000
)

if (!(Test-Path -LiteralPath $File)) {
  Write-Error "File not found: $File"
  exit 1
}
$fi = Get-Item -LiteralPath $File
$size = [uint64]$fi.Length
if ($size -eq 0) { Write-Error "Zero-length file"; exit 1 }

# CRC32 (IEEE 802.3, polynomial 0xEDB88320)
function New-Crc32Table {
  $t = New-Object 'uint32[]' 256
  for ($i=0; $i -lt 256; $i++) {
    $c = [uint32]$i
    for ($j=0; $j -lt 8; $j++) {
      if (($c -band 1u) -ne 0u) {
        $c = (0xEDB88320u -bxor ($c -shr 1))
      } else {
        $c = ($c -shr 1)
      }
    }
    $t[$i] = $c
  }
  return ,$t
}
$CRC32_TABLE = New-Crc32Table
function Get-Crc32([byte[]]$buf, [int]$off, [int]$len) {
  $crc = 0xffffffffu
  for ($i = 0; $i -lt $len; $i++) {
    $idx = ($crc -bxor $buf[$off + $i]) -band 0xffu
    $crc = ($CRC32_TABLE[$idx] -bxor ($crc -shr 8))
  }
  return (-bnot $crc) -band 0xffffffffu
}

# Frame builder
$MAGIC = [byte[]](0xA5,0x5A,0x4B,0x52)
function Make-Frame([uint32]$offset, [byte[]]$payload, [int]$plen) {
  $hdr = New-Object 'byte[]' (16 + $plen)
  [Array]::Copy($MAGIC, 0, $hdr, 0, 4)
  [Array]::Copy([BitConverter]::GetBytes([uint32]$offset), 0, $hdr, 4, 4)
  [Array]::Copy([BitConverter]::GetBytes([uint32]$plen  ), 0, $hdr, 8, 4)
  $crc = if ($plen -gt 0) { Get-Crc32 $payload 0 $plen } else { 0u }
  [Array]::Copy([BitConverter]::GetBytes([uint32]$crc), 0, $hdr, 12, 4)
  if ($plen -gt 0) { [Array]::Copy($payload, 0, $hdr, 16, $plen) }
  return $hdr
}

function Read-LineWithTimeout([System.IO.Ports.SerialPort]$sp, [int]$timeoutMs) {
  $sw = [System.Diagnostics.Stopwatch]::StartNew()
  $acc = New-Object System.Text.StringBuilder
  while ($sw.ElapsedMilliseconds -lt $timeoutMs) {
    try {
      if ($sp.BytesToRead -gt 0) {
        $s = $sp.ReadExisting()
        [void]$acc.Append($s)
        $txt = $acc.ToString()
        $pos = $txt.IndexOf("`n")
        if ($pos -ge 0) {
          $line = $txt.Substring(0, $pos).Trim()
          return $line
        }
      } else {
        Start-Sleep -Milliseconds 10
      }
    } catch {
      Start-Sleep -Milliseconds 5
    }
  }
  throw "Timeout waiting for line"
}

# Open serial
$sp = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$sp.ReadTimeout = 500
$sp.WriteTimeout = 5000
$sp.NewLine = "`n"
$sp.Open()
Start-Sleep -Milliseconds 200

# Flush input
while ($sp.BytesToRead -gt 0) { [void]$sp.ReadExisting(); Start-Sleep -Milliseconds 10 }

# Send command
$cmd = "putbin $Name $size`r`n"
$sp.Write($cmd)
Write-Host "Sent: $($cmd.Trim())"

# Wait READY
do {
  $line = Read-LineWithTimeout $sp $LineTimeoutMs
  Write-Host "< $line"
} while ($line -notmatch '^READY($|\s)')

# Stream file
$fs = [System.IO.File]::OpenRead($File)
$buf = New-Object 'byte[]' $Chunk
$sent = [uint64]0
$start = Get-Date

while ($sent -lt $size) {
  $toRead = [int][Math]::Min([uint64]$Chunk, $size - $sent)
  $rd = $fs.Read($buf, 0, $toRead)
  if ($rd -le 0) { throw "Short read" }
  $frame = Make-Frame([uint32]$sent, $buf, $rd)
  $sp.Write($frame, 0, $frame.Length)
  $sent += [uint64]$rd

  if (($sent % (512*1024)) -eq 0 -or $sent -eq $size) {
    $secs = ([DateTime]::Now - $start).TotalSeconds
    if ($secs -le 0) { $secs = 0.001 }
    $mb = [Math]::Round($sent / 1MB, 2)
    $spd = [Math]::Round(($sent / 1MB) / $secs, 2)
    Write-Host ("  {0} MiB sent  ({1} MiB/s)" -f $mb, $spd)
  }
}

$fs.Dispose()

# Commit
$commit = Make-Frame(0xFFFFFFFF, $buf, 0)
$sp.Write($commit, 0, $commit.Length)
Write-Host "Commit sent, awaiting OK..."

for (;;) {
  $line = Read-LineWithTimeout $sp 20000
  Write-Host "< $line"
  if ($line -match '^OK($|\s)') { break }
  if ($line -match '^ERR') { throw "Device reported error" }
}

$sp.Close()
Write-Host "Done."