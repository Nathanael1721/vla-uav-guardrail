<#
.SYNOPSIS
    Turn a report HTML (tools/report_to_html.py) into .docx and .pdf with Word,
    with every figure embedded in the .docx.

.DESCRIPTION
    Three short Word sessions, each retried, instead of one long one:

      1. open HTML            -> SaveAs .docx (figures still linked)
      2. open .docx           -> embed + break each figure link -> Save
      3. open .docx           -> SaveAs .pdf

    Why, from tracing it on 2026-09-14. Inside this OneDrive-synced repo the
    Word COM server disconnected (RPC_E_DISCONNECTED) intermittently: the first
    run after a quiet spell succeeded, and a run within a minute or two of the
    previous one failed - at a different step each time - while OneDrive was
    still uploading the last outputs (it also restored build files as they were
    deleted). Working on a copy outside the synced folder made three
    back-to-back runs succeed on their first attempt. The retry loop stays as a
    guard, not as the fix.

    Also observed and avoided:

      * breaking figure links on the HTML-origin document made Word disconnect on
        Close - so links are broken on the reopened .docx;
      * opening the HTML read-only made Close disconnect on 4 of 4 attempts;
      * a work folder under %TEMP% wrote nothing: Word opens Temp files in
        Protected View, and a hidden instance then cannot return the document;
      * the final files are only replaced once all three steps succeeded, so a
        failed run never deletes the last good .docx/.pdf.

.EXAMPLE
    powershell -File tools\html_to_docx_pdf.ps1 -Html docs\MID-EVALUATION-REPORT-Sep2026.html
#>
param([Parameter(Mandatory = $true)][string]$Html, [int]$Tries = 4)
$ErrorActionPreference = "Stop"

$full = (Resolve-Path -LiteralPath $Html).Path
$finalDocx = [System.IO.Path]::ChangeExtension($full, ".docx")
$finalPdf  = [System.IO.Path]::ChangeExtension($full, ".pdf")
# Word works on a COPY of the HTML in a folder that is neither OneDrive-synced
# nor %TEMP% (Word opens Temp files in Protected View). Inside the synced repo,
# the first run after a quiet spell succeeded and any run within a minute or two
# of the previous one disconnected, while OneDrive was still uploading the last
# outputs and restoring deleted build files. Figures are absolute file:// URIs,
# so the copy renders identically.
$work = Join-Path $env:USERPROFILE (".guardrail_word_build\" + [guid]::NewGuid().ToString("N").Substring(0, 8))
New-Item -ItemType Directory -Force $work | Out-Null
$src  = [string](Join-Path $work ([System.IO.Path]::GetFileName($full)))
Copy-Item -LiteralPath $full -Destination $src -Force
$docx = [System.IO.Path]::ChangeExtension($src, ".docx")
$pdf  = [System.IO.Path]::ChangeExtension($src, ".pdf")
$started = Get-Date
function step($m) { Write-Host ("    [{0:HH:mm:ss}] {1}" -f (Get-Date), $m) }

# Word processes that belong to the user, not to this script. Never touched.
$baseline = @(Get-Process WINWORD -ErrorAction SilentlyContinue | ForEach-Object Id)

# New-Object straight after Quit() can attach to a Word server that is still
# shutting down. Not the main cause here (that was OneDrive, see the header),
# but waiting for our own previous instance to exit is cheap and removes it.
function Wait-WordExited([int]$timeoutS = 45) {
    $t0 = Get-Date
    while (@(Get-Process WINWORD -ErrorAction SilentlyContinue | Where-Object { $baseline -notcontains $_.Id }).Count -gt 0) {
        if (((Get-Date) - $t0).TotalSeconds -gt $timeoutS) { step "an automation Word instance is still running after ${timeoutS} s"; return }
        Start-Sleep -Milliseconds 500
    }
}

# Run one Word session; retry on a fresh instance if the COM server drops.
function In-Word([string]$what, [scriptblock]$body) {
    for ($k = 1; $k -le $Tries; $k++) {
        Wait-WordExited
        $app = New-Object -ComObject Word.Application
        $app.Visible = $false
        try {
            & $body $app
            step "$what (attempt $k)"
            return
        } catch {
            step "$what failed on attempt ${k}: $($_.Exception.Message)"
            if ($k -eq $Tries) { throw }
            Start-Sleep -Seconds 3
        } finally {
            try { $app.Quit() } catch { }
            try { [void][System.Runtime.InteropServices.Marshal]::ReleaseComObject($app) } catch { }
        }
    }
}

Write-Host "==> Word (from HTML): $([System.IO.Path]::GetFileName($full))"

In-Word "saved .docx, figures linked" {
    param($app)
    if (Test-Path -LiteralPath $docx) { Remove-Item -LiteralPath $docx -Force }
    # Read-write on purpose: opening the HTML read-only made Close disconnect
    # on 4 of 4 attempts (2026-09-14).
    $doc = $app.Documents.Open($src, $false, $false)
    $doc.SaveAs([ref]$docx, [ref]16)          # 16 = wdFormatDocumentDefault
    $doc.Close($false)
}

In-Word "embedded figures in .docx" {
    param($app)
    $doc = $app.Documents.Open($docx, $false, $false)
    foreach ($shape in @($doc.InlineShapes)) {
        if ($shape.LinkFormat -ne $null) {
            $shape.LinkFormat.SavePictureWithDocument = $true
            $shape.LinkFormat.BreakLink()
        }
    }
    $left = @($doc.InlineShapes | Where-Object { $_.LinkFormat -ne $null }).Count
    if ($left -ne 0) { throw "$left figure link(s) still external" }
    $doc.Save()
    $doc.Close($false)
}

In-Word "exported .pdf" {
    param($app)
    if (Test-Path -LiteralPath $pdf) { Remove-Item -LiteralPath $pdf -Force }
    $doc = $app.Documents.Open($docx, $false, $false)
    $doc.SaveAs([ref]$pdf, [ref]17)           # 17 = wdFormatPDF
    $doc.Close($false)
}

# Do not return while our last Word instance is still exiting: a caller that
# starts Word again immediately would attach to it and hit the same race.
Wait-WordExited

foreach ($f in @($docx, $pdf)) {
    $i = Get-Item -LiteralPath $f
    if ($i.Length -lt 1024 -or $i.LastWriteTime -lt $started) { throw "stale or empty: $f" }
}
Copy-Item -LiteralPath $docx -Destination $finalDocx -Force
Copy-Item -LiteralPath $pdf -Destination $finalPdf -Force
Remove-Item -LiteralPath $work -Recurse -Force
foreach ($f in @($finalDocx, $finalPdf)) {
    $i = Get-Item -LiteralPath $f
    Write-Host ("    {0,-52} {1,8:N2} MB" -f $i.Name, ($i.Length / 1MB))
}
