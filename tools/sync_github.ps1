param(
    [Parameter(Mandatory = $true)]
    [string]$Message
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $projectRoot
$publicationRoot = Join-Path $workspaceRoot '.github-publish\acg720-vision-robot'
$remoteUrl = & git -c "safe.directory=$projectRoot" -C $projectRoot remote get-url origin
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($remoteUrl)) {
    throw '未配置 origin；请先连接 GitHub 仓库。'
}

# Keep full local vendor history out of the public source repository.
# There is no deletion, clean, reset, forced update or token handling here.
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $publicationRoot) | Out-Null
if (-not (Test-Path -LiteralPath (Join-Path $publicationRoot '.git'))) {
    if (Test-Path -LiteralPath $publicationRoot) {
        throw "发布目录已存在但不是 Git 仓库，保留现场：$publicationRoot"
    }
    & git clone $remoteUrl $publicationRoot
    if ($LASTEXITCODE -ne 0) { throw 'git clone 失败；检查网络和 GitHub 登录。' }
}

function Invoke-PublishGit {
    & git -c "safe.directory=$publicationRoot" -C $publicationRoot @args
    if ($LASTEXITCODE -ne 0) { throw "发布目录 Git 操作失败：$args" }
}

$publishRemote = Invoke-PublishGit remote get-url origin
if ($publishRemote.TrimEnd('/') -ne $remoteUrl.TrimEnd('/')) {
    throw '发布目录的 origin 与当前工程不同，已停止。'
}
$publishBranch = Invoke-PublishGit branch --show-current
if ($publishBranch -ne 'main') { throw '发布目录必须在 main 分支，已停止。' }
$publishDirty = Invoke-PublishGit status --porcelain
if ($publishDirty) { throw '发布目录有未提交修改，已停止并保留现场。' }
Invoke-PublishGit fetch origin main
Invoke-PublishGit pull --ff-only origin main

$publishFiles = [Collections.Generic.List[string]]::new()
foreach ($name in @('README.md', '.gitignore')) {
    $publishFiles.Add($name)
}
Get-ChildItem -LiteralPath $projectRoot -File -Filter '*.gprj' |
    ForEach-Object { $publishFiles.Add($_.Name) }
foreach ($folder in @('pc', 'docs', 'tests', 'tools')) {
    $folderPath = Join-Path $projectRoot $folder
    Get-ChildItem -LiteralPath $folderPath -File -Recurse |
        Where-Object {
            $_.FullName -notmatch '[\\/]__pycache__[\\/]' -and
            $_.Extension -in @('.py', '.md', '.svg', '.txt', '.cmd', '.ps1', '.sh')
        } | ForEach-Object {
            $publishFiles.Add($_.FullName.Substring($projectRoot.Length + 1))
        }
}
Get-ChildItem -LiteralPath (Join-Path $projectRoot 'src') -File |
    Where-Object { $_.Extension -in @('.v', '.cst', '.sdc') } |
    ForEach-Object { $publishFiles.Add('src\' + $_.Name) }
foreach ($name in @('vision_robot_process_config.json',
                    'vision_robot_netfix_process_config.json',
                    'vision_robot_ui_process_config.json')) {
    if (Test-Path -LiteralPath (Join-Path $projectRoot ('impl\' + $name))) {
        $publishFiles.Add('impl\' + $name)
    }
}

foreach ($relativeFile in $publishFiles) {
    $sourceFile = Join-Path $projectRoot $relativeFile
    $targetFile = Join-Path $publicationRoot $relativeFile
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $targetFile) | Out-Null
    Copy-Item -LiteralPath $sourceFile -Destination $targetFile -Force
}
Invoke-PublishGit add --all
& git -c "safe.directory=$publicationRoot" -C $publicationRoot diff --cached --quiet
if ($LASTEXITCODE -eq 0) {
    Write-Host '自编文件没有新变更。'
    exit 0
}
if ($LASTEXITCODE -ne 1) { throw '无法检查暂存变更。' }
Invoke-PublishGit commit -m $Message
Invoke-PublishGit push origin main
Write-Host '自编源码已同步；厂商依赖、位流、照片仍保留在本地。'
