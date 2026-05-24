# IP Term Project - CPLEX Full Version

這是 C# + CPLEX 完整版。

## 功能

- 讀取五個檔案：
  - ORDER.csv
  - MACHINE.csv
  - MOLD.csv
  - MATERIAL_ROUTE.csv
  - MAINTENANCE.csv

- 自動產生：
  - single-order groups
  - partial cavity groups

- CPLEX MILP 決定：
  - 哪些 groups 要選
  - 每個 group 的 start time
  - 同一台 machine 上 group 的先後順序
  - maintenance 前後 disjunctive 排程
  - mold conflict no-overlap

- 輸出：
  - result.csv
  - validation_report.txt

## 執行方式

把五個 CSV 放在同一個資料夾，例如：

```text
C:\IP_Project\Input2\
├── ORDER.csv
├── MACHINE.csv
├── MOLD.csv
├── MATERIAL_ROUTE.csv
└── MAINTENANCE.csv
```

然後執行：

```bash
dotnet run -- "C:\IP_Project\Input2" 2 120 6000
```

參數：

```text
第 1 個參數：input folder
第 2 個參數：max partial group size，建議先用 2
第 3 個參數：CPLEX time limit seconds
第 4 個參數：max candidate groups
```

建議先跑：

```bash
dotnet run -- "C:\IP_Project\Input2" 2 120 6000
```

如果 Input 3 想試三張一組：

```bash
dotnet run -- "C:\IP_Project\Input3" 3 120 8000
```

## CPLEX DLL 設定

專案使用環境變數：

```text
CPLEX_STUDIO_DIR
```

例如你的 CPLEX 裝在：

```text
C:\Program Files\IBM\ILOG\CPLEX_Studio2211
```

請在 PowerShell 執行：

```powershell
setx CPLEX_STUDIO_DIR "C:\Program Files\IBM\ILOG\CPLEX_Studio2211"
```

重新開啟 terminal 後再 build。

或是直接打開 `.csproj`，把路徑改成你的實際 DLL 位置：

```xml
<HintPath>C:\Program Files\IBM\ILOG\CPLEX_Studio2211\concert\bin\x64_win64\ILOG.Concert.dll</HintPath>
<HintPath>C:\Program Files\IBM\ILOG\CPLEX_Studio2211\cplex\bin\x64_win64\ILOG.CPLEX.dll</HintPath>
```

## 注意事項

這版的 CPLEX 模型採用 candidate group model：

- 先產生很多 feasible candidate groups
- 再用 MILP 從其中挑選排程

這比直接把所有 order sequencing 全部丟進 CPLEX 穩定很多。

模型中 mold availability 使用 pairwise conflict constraint：
如果兩個 groups 同時使用同一種 mold 會超過數量，就強制兩者不可重疊。

最後 `validation_report.txt` 會完整重檢：
- duplicate order
- due date
- route
- capacity
- cavity
- same mold type in group
- machine overlap
- maintenance
- mold availability
- mold change limit
