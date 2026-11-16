# Launch USD Composer with the Factory Work Cell extension and Isaac Sim's extensions (cuMotion, asset browsers).
#   .\run_composer.ps1            open the app
#   .\run_composer.ps1 -Smoke     open the app and run tests\kit\kit_smoke.py in it, visibly. The app stays open.
# [SMOKE] PASS/FAIL lines print in this terminal. Close the Composer window to end.
#   .\run_composer.ps1 -Exec tests\kit\kit_robot.py    same, for any in-app test script (UR10 + cuMotion here)
param(
    [switch]$Smoke,
    [string]$Exec = '',
    [string]$Composer = 'C:\Omniverse\kit-app-template\_build\windows-x86_64\release',
    [string]$Isaac = 'C:\isaacsim\env\Lib\site-packages\isaacsim'
)
$repo = Split-Path $PSScriptRoot -Parent
$kitArgs = @(
    "$Composer\apps\my_company.my_usd_composer.kit",
    '--ext-folder', "$Composer\exts", '--ext-folder', "$Composer\extscache", '--ext-folder', "$Composer\apps",
    '--ext-folder', "$Isaac\exts", '--ext-folder', "$Isaac\extscache",
    '--ext-folder', "$repo\omniverse", '--ext-folder', "$repo",  # $repo: finds simcore\extension.toml
    '--enable', 'factory.workcell', '--enable', 'omni.kit.browser.asset', '--enable', 'omni.simready.explorer',
    '--enable', 'isaacsim.gui.content_browser',  # adds NVIDIA's Isaac Sim asset folders to the Content browser
    '--/persistent/app/stage/upAxis=Z',  # new files start Z-up like the plant, so only the units switch on first add
    '--/persistent/isaac/asset_root/default=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2'
)
# Optional: Omniverse MCP bridge (serionist/omniverse-mcp), for scripted control of the open stage. Port 8211, token
# auth. Patched copy (Kit 110.4 skips the shipped one because of its kit = ["107.0"] target line). Skipped if absent.
if (Test-Path 'C:\Omniverse\ovmcp_exts\ovmcp.bridge') {
    $kitArgs += @('--ext-folder', 'C:\Omniverse\ovmcp_exts', '--enable', 'ovmcp.bridge')
}
if ($Smoke) { $Exec = 'tests\kit\kit_smoke.py' }
if ($Exec) { $kitArgs += @('--exec', (Join-Path $repo $Exec)) }
& "$Composer\kit\kit.exe" @kitArgs
