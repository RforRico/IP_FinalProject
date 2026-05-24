using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using ILOG.Concert;
using ILOG.CPLEX;

// IP Term Project - CPLEX Full Version
//
// 功能：
// 1. 讀取 ORDER / MACHINE / MOLD / MATERIAL_ROUTE / MAINTENANCE
// 2. 自動產生 feasible production groups，包含 partial cavity group
// 3. 使用 CPLEX MILP 決定：
//    - 哪些 candidate groups 被選
//    - 每個 group 的 processing start time
//    - 同一機台上的 group 先後順序
//    - maintenance 前後的 disjunctive 排程
//    - mold conflict group 的 no-overlap
// 4. 輸出 result.csv
// 5. 輸出 validation_report.txt
//
// 執行範例：
// dotnet run -- "C:\IP_Project\Input2" 2 120 6000
//
// args[0] = input folder
// args[1] = max partial group size，建議先用 2
// args[2] = CPLEX time limit seconds，預設 120
// args[3] = max candidate groups，預設 6000

class Program
{
    static readonly DateTime BaseTime = DateTime.ParseExact(
        "2026/05/01 08:00:00",
        "yyyy/MM/dd HH:mm:ss",
        CultureInfo.InvariantCulture
    );

    const int MountMinutes = 30;
    const int UnmountMinutes = 20;
    const int CleanMinutes = 15;
    const int TuneMinutes = 10;

    const double FirstPreSetupSeconds = (MountMinutes + TuneMinutes) * 60.0;       // 40 min
    const double BetweenSetupSeconds = (UnmountMinutes + TuneMinutes + MountMinutes) * 60.0; // 60 min
    const double BetweenBeforeMaintenanceSeconds = (UnmountMinutes + TuneMinutes) * 60.0;     // 30 min
    const double BetweenAfterMaintenanceSeconds = MountMinutes * 60.0;             // 30 min
    const double LastPostSetupSeconds = UnmountMinutes * 60.0;                    // 20 min

    static void Main(string[] args)
    {
        Stopwatch totalWatch = new Stopwatch();
        totalWatch.Start();

        string folder = args.Length >= 1 ? args[0] : Directory.GetCurrentDirectory();
        int maxGroupSize = args.Length >= 2 ? int.Parse(args[1]) : 2;
        double timeLimitSeconds = args.Length >= 3 ? double.Parse(args[2], CultureInfo.InvariantCulture) : 120.0;
        int maxCandidates = args.Length >= 4 ? int.Parse(args[3]) : 6000;

        Console.WriteLine("===== IP Term Project CPLEX Solver =====");
        Console.WriteLine("Input folder       : " + folder);
        Console.WriteLine("Max group size     : " + maxGroupSize);
        Console.WriteLine("CPLEX time limit   : " + timeLimitSeconds + " sec");
        Console.WriteLine("Max candidate count: " + maxCandidates);
        Console.WriteLine();

        InputData data = ReadAllData(folder);

        Console.WriteLine("Orders      : " + data.Orders.Count);
        Console.WriteLine("Machines    : " + data.Machines.Count);
        Console.WriteLine("Molds       : " + data.Molds.Count);
        Console.WriteLine("Routes      : " + data.Routes.Count);
        Console.WriteLine("Maintenance : " + data.Maintenances.Count);
        Console.WriteLine();

        List<CandidateGroup> candidates = GenerateCandidateGroups(data, maxGroupSize, maxCandidates);

        Console.WriteLine("Candidate groups generated: " + candidates.Count);
        PrintCandidateSummary(candidates);
        Console.WriteLine();

        SolveResult solveResult = SolveByCplex(data, candidates, timeLimitSeconds);

        totalWatch.Stop();

        List<ResultRow> rows = BuildResultRows(data, solveResult);
        int objValue = rows.Count(r => r.IsScheduled == 1);

        string resultPath = Path.Combine(folder, "result.csv");
        WriteResultCsv(resultPath, rows, totalWatch.Elapsed.TotalSeconds, objValue);

        string reportPath = Path.Combine(folder, "validation_report.txt");
        List<string> validationReport = ValidateSolution(data, solveResult, rows);
        File.WriteAllLines(reportPath, validationReport);

        Console.WriteLine();
        Console.WriteLine("===== Final Summary =====");
        Console.WriteLine("CPLEX status : " + solveResult.CplexStatus);
        Console.WriteLine("ObjValue     : " + objValue + " / " + data.Orders.Count);
        Console.WriteLine("Total runtime: " + totalWatch.Elapsed.TotalSeconds.ToString("0.000", CultureInfo.InvariantCulture) + " sec");
        Console.WriteLine("Result file  : " + resultPath);
        Console.WriteLine("Report file  : " + reportPath);
        Console.WriteLine();

        foreach (string line in validationReport)
        {
            Console.WriteLine(line);
        }
    }

    // ============================================================
    // CPLEX model
    // ============================================================

    static SolveResult SolveByCplex(InputData data, List<CandidateGroup> candidates, double timeLimitSeconds)
    {
        SolveResult result = new SolveResult();

        int G = candidates.Count;
        int O = data.Orders.Count;

        List<int> orderNos = data.Orders.Keys.OrderBy(x => x).ToList();
        Dictionary<int, int> orderIndex = new Dictionary<int, int>();
        for (int i = 0; i < orderNos.Count; i++)
        {
            orderIndex[orderNos[i]] = i;
        }

        double horizon = EstimateHorizonSeconds(data, candidates);
        double bigM = horizon + 30.0 * 24.0 * 3600.0;

        Console.WriteLine("Estimated horizon seconds: " + horizon.ToString("0", CultureInfo.InvariantCulture));
        Console.WriteLine("Big-M seconds            : " + bigM.ToString("0", CultureInfo.InvariantCulture));
        Console.WriteLine();

        Cplex model = new Cplex();

        try
        {
            model.SetParam(Cplex.Param.TimeLimit, timeLimitSeconds);
            model.SetParam(Cplex.Param.RandomSeed, 1);
            model.SetParam(Cplex.Param.Threads, 0);
            model.SetParam(Cplex.Param.MIP.Display, 2);

            INumVar[] y = new INumVar[G];
            INumVar[] start = new INumVar[G];

            for (int g = 0; g < G; g++)
            {
                y[g] = model.BoolVar("y_" + g);
                start[g] = model.NumVar(0.0, horizon, NumVarType.Float, "s_" + g);
            }

            INumVar[] u = new INumVar[O];
            for (int i = 0; i < O; i++)
            {
                u[i] = model.BoolVar("u_" + orderNos[i]);
            }

            // Objective:
            // 第一優先：最大化 scheduled orders
            // 第二優先：偏好較早開始
            // 第三優先：稍微懲罰 group 數，避免切太碎
            ILinearNumExpr objective = model.LinearNumExpr();

            for (int i = 0; i < O; i++)
            {
                objective.AddTerm(1000000.0, u[i]);
            }

            for (int g = 0; g < G; g++)
            {
                objective.AddTerm(-0.0001, start[g]);
                objective.AddTerm(-1.0, y[g]);
            }

            model.AddMaximize(objective);

            // ------------------------------------------------------------
            // Constraint 1:
            // 每張訂單最多被一個 selected group 覆蓋，且 u_i = sum selected group containing i
            // ------------------------------------------------------------
            for (int i = 0; i < O; i++)
            {
                int orderNo = orderNos[i];

                ILinearNumExpr expr = model.LinearNumExpr();

                for (int g = 0; g < G; g++)
                {
                    if (candidates[g].OrderNos.Contains(orderNo))
                    {
                        expr.AddTerm(1.0, y[g]);
                    }
                }

                expr.AddTerm(-1.0, u[i]);
                model.AddEq(expr, 0.0, "order_cover_" + orderNo);
            }

            // ------------------------------------------------------------
            // Constraint 2:
            // 每台機器 selected group 數不得超過 mold change limit
            // ------------------------------------------------------------
            foreach (Machine m in data.Machines.Values)
            {
                ILinearNumExpr expr = model.LinearNumExpr();

                for (int g = 0; g < G; g++)
                {
                    if (candidates[g].MachineNo == m.MachineNo)
                    {
                        expr.AddTerm(1.0, y[g]);
                    }
                }

                model.AddLe(expr, m.MoldChangeLimit, "mold_change_limit_" + m.MachineNo);
            }

            // ------------------------------------------------------------
            // Constraint 3:
            // 若 group 被選，processing start 必須至少在 40 min 之後，
            // 因為第一個 group 之前需要 Mount + Tune。
            // s_g >= 40min if y_g = 1
            // ------------------------------------------------------------
            for (int g = 0; g < G; g++)
            {
                ILinearNumExpr expr = model.LinearNumExpr();
                expr.AddTerm(-1.0, start[g]);
                expr.AddTerm(bigM, y[g]);
                model.AddLe(expr, bigM - FirstPreSetupSeconds, "first_setup_lb_" + g);
            }

            // ------------------------------------------------------------
            // Constraint 4:
            // Due date constraint
            // 若 group g 被選，group 中每張訂單都必須在 due date 前完成。
            // start_g + offsetEnd_ig <= due_i
            // ------------------------------------------------------------
            for (int g = 0; g < G; g++)
            {
                CandidateGroup cg = candidates[g];

                foreach (int orderNo in cg.OrderNos)
                {
                    Order o = data.Orders[orderNo];
                    double dueSec = (o.DueDate - BaseTime).TotalSeconds;
                    double endOffset = cg.OrderSegments[orderNo].EndOffsetSeconds;

                    ILinearNumExpr expr = model.LinearNumExpr();
                    expr.AddTerm(1.0, start[g]);
                    expr.AddTerm(bigM, y[g]);

                    model.AddLe(expr, dueSec - endOffset + bigM, "due_g" + g + "_o" + orderNo);
                }
            }

            // ------------------------------------------------------------
            // Constraint 5:
            // 同一台 machine 上的 selected groups 不可重疊，且中間要有 60 min setup。
            // ------------------------------------------------------------
            int machinePairCount = 0;

            for (int a = 0; a < G; a++)
            {
                for (int b = a + 1; b < G; b++)
                {
                    if (candidates[a].MachineNo != candidates[b].MachineNo) continue;

                    INumVar before = model.BoolVar("before_m_" + a + "_" + b);

                    AddPairNoOverlap(
                        model,
                        start[a],
                        start[b],
                        y[a],
                        y[b],
                        before,
                        candidates[a].TotalSpanSeconds,
                        candidates[b].TotalSpanSeconds,
                        BetweenSetupSeconds,
                        bigM,
                        "machine_pair_" + a + "_" + b
                    );

                    machinePairCount++;
                }
            }

            Console.WriteLine("Machine sequencing pair constraints: " + machinePairCount);

            // ------------------------------------------------------------
            // Constraint 6:
            // Maintenance disjunction
            //
            // 這裡採保守但安全的 block：
            // [processing start - 40min, processing end + 20min]
            // 不可與 maintenance overlap。
            //
            // 代表 setup 和 processing 都不會碰到 maintenance。
            // ------------------------------------------------------------
            int maintenanceConstraintCount = 0;

            for (int g = 0; g < G; g++)
            {
                CandidateGroup cg = candidates[g];

                List<Maintenance> maints = data.MaintenancesByMachine.ContainsKey(cg.MachineNo)
                    ? data.MaintenancesByMachine[cg.MachineNo]
                    : new List<Maintenance>();

                foreach (Maintenance mt in maints)
                {
                    INumVar beforeMaintenance = model.BoolVar("before_mt_" + g + "_" + maintenanceConstraintCount);

                    double mtStartSec = (mt.StartTime - BaseTime).TotalSeconds;
                    double mtEndSec = (mt.EndTime - BaseTime).TotalSeconds;

                    // If beforeMaintenance = 1:
                    // start_g + span_g + post <= maintenance start
                    {
                        ILinearNumExpr expr = model.LinearNumExpr();
                        expr.AddTerm(1.0, start[g]);
                        expr.AddTerm(bigM, beforeMaintenance);
                        expr.AddTerm(bigM, y[g]);

                        double rhs = mtStartSec + 2.0 * bigM - cg.TotalSpanSeconds - LastPostSetupSeconds;
                        model.AddLe(expr, rhs, "maintenance_before_" + g + "_" + maintenanceConstraintCount);
                    }

                    // If beforeMaintenance = 0:
                    // start_g - pre >= maintenance end
                    {
                        ILinearNumExpr expr = model.LinearNumExpr();
                        expr.AddTerm(-1.0, start[g]);
                        expr.AddTerm(-bigM, beforeMaintenance);
                        expr.AddTerm(bigM, y[g]);

                        double rhs = -mtEndSec + bigM - FirstPreSetupSeconds;
                        model.AddLe(expr, rhs, "maintenance_after_" + g + "_" + maintenanceConstraintCount);
                    }

                    maintenanceConstraintCount++;
                }
            }

            Console.WriteLine("Maintenance disjunctions: " + maintenanceConstraintCount);

            // ------------------------------------------------------------
            // Constraint 7:
            // Mold availability 的 pairwise conflict。
            //
            // 若兩個 groups 使用同一 mold，且兩者同時使用會超過 mold quantity，
            // 則這兩個 groups 不可重疊。
            //
            // 注意：
            // 這是 pairwise conflict 模型，對大部分作業資料已夠用。
            // 若要完全處理多個 group 同時累積超量，需建立更大的 cumulative model。
            // 但後面的 validator 仍會完整重檢 mold availability。
            // ------------------------------------------------------------
            int moldPairCount = 0;

            for (int a = 0; a < G; a++)
            {
                for (int b = a + 1; b < G; b++)
                {
                    if (candidates[a].MachineNo == candidates[b].MachineNo) continue;

                    if (!NeedMoldNoOverlap(data, candidates[a], candidates[b])) continue;

                    INumVar before = model.BoolVar("before_q_" + a + "_" + b);

                    AddPairNoOverlap(
                        model,
                        start[a],
                        start[b],
                        y[a],
                        y[b],
                        before,
                        candidates[a].TotalSpanSeconds,
                        candidates[b].TotalSpanSeconds,
                        0.0,
                        bigM,
                        "mold_pair_" + a + "_" + b
                    );

                    moldPairCount++;
                }
            }

            Console.WriteLine("Mold conflict pair constraints: " + moldPairCount);
            Console.WriteLine();

            bool solved = model.Solve();

            result.CplexSolved = solved;
            result.CplexStatus = model.GetStatus().ToString();

            if (solved)
            {
                Console.WriteLine("CPLEX status  : " + model.GetStatus());
                Console.WriteLine("CPLEX obj     : " + model.GetObjValue().ToString("0.###", CultureInfo.InvariantCulture));
                Console.WriteLine("CPLEX best bd : " + model.GetBestObjValue().ToString("0.###", CultureInfo.InvariantCulture));
                Console.WriteLine();

                List<ScheduledGroup> selected = new List<ScheduledGroup>();

                for (int g = 0; g < G; g++)
                {
                    double yg = model.GetValue(y[g]);

                    if (yg >= 0.5)
                    {
                        double st = model.GetValue(start[g]);
                        CandidateGroup cg = candidates[g];

                        ScheduledGroup sg = new ScheduledGroup();
                        sg.Candidate = cg;
                        sg.MachineNo = cg.MachineNo;
                        sg.ProcessingStart = BaseTime.AddSeconds(st);
                        sg.ProcessingEnd = sg.ProcessingStart.AddSeconds(cg.TotalSpanSeconds);

                        selected.Add(sg);
                    }
                }

                result.Groups = selected
                    .OrderBy(g => g.MachineNo)
                    .ThenBy(g => g.ProcessingStart)
                    .ToList();

                result.ModelObjectiveValue = model.GetObjValue();
                result.BestBound = model.GetBestObjValue();
            }
            else
            {
                Console.WriteLine("CPLEX did not find a feasible solution.");
                result.Groups = new List<ScheduledGroup>();
            }
        }
        finally
        {
            model.End();
        }

        return result;
    }

    static void AddPairNoOverlap(
        Cplex model,
        INumVar startA,
        INumVar startB,
        INumVar yA,
        INumVar yB,
        INumVar beforeAB,
        double spanA,
        double spanB,
        double setupSeconds,
        double bigM,
        string namePrefix)
    {
        // beforeAB = 1 -> A before B
        // startA + spanA + setup <= startB
        {
            ILinearNumExpr expr = model.LinearNumExpr();
            expr.AddTerm(1.0, startA);
            expr.AddTerm(-1.0, startB);
            expr.AddTerm(bigM, beforeAB);
            expr.AddTerm(bigM, yA);
            expr.AddTerm(bigM, yB);

            double rhs = 3.0 * bigM - spanA - setupSeconds;
            model.AddLe(expr, rhs, namePrefix + "_A_before_B");
        }

        // beforeAB = 0 -> B before A
        // startB + spanB + setup <= startA
        {
            ILinearNumExpr expr = model.LinearNumExpr();
            expr.AddTerm(1.0, startB);
            expr.AddTerm(-1.0, startA);
            expr.AddTerm(-bigM, beforeAB);
            expr.AddTerm(bigM, yA);
            expr.AddTerm(bigM, yB);

            double rhs = 2.0 * bigM - spanB - setupSeconds;
            model.AddLe(expr, rhs, namePrefix + "_B_before_A");
        }
    }

    static bool NeedMoldNoOverlap(InputData data, CandidateGroup a, CandidateGroup b)
    {
        foreach (string moldNo in a.MoldUsage.Keys)
        {
            if (!b.MoldUsage.ContainsKey(moldNo)) continue;
            if (!data.Molds.ContainsKey(moldNo)) return true;

            int combined = a.MoldUsage[moldNo] + b.MoldUsage[moldNo];
            int available = data.Molds[moldNo].Qty;

            if (combined > available)
            {
                return true;
            }
        }

        return false;
    }

    static double EstimateHorizonSeconds(InputData data, List<CandidateGroup> candidates)
    {
        double maxDue = data.Orders.Values.Max(o => (o.DueDate - BaseTime).TotalSeconds);
        double totalProcessing = data.Orders.Values.Sum(o => o.ProcessTimeSeconds) * 3.0;
        double setupAllowance = data.Orders.Count * BetweenSetupSeconds + 7.0 * 24.0 * 3600.0;

        double candidateMax = candidates.Count > 0 ? candidates.Max(c => c.TotalSpanSeconds) : 0.0;

        return Math.Max(maxDue + 7.0 * 24.0 * 3600.0, totalProcessing + setupAllowance + candidateMax);
    }

    // ============================================================
    // Candidate group generation
    // ============================================================

    static List<CandidateGroup> GenerateCandidateGroups(InputData data, int maxGroupSize, int maxCandidates)
    {
        List<CandidateGroup> candidates = new List<CandidateGroup>();

        List<Order> orders = data.Orders.Values
            .OrderBy(o => o.DueDate)
            .ThenBy(o => o.OrderNo)
            .ToList();

        int tempId = 1;

        for (int size = 1; size <= maxGroupSize; size++)
        {
            foreach (List<Order> subset in Combinations(orders, size))
            {
                List<List<Order>> sequences;

                if (size <= 2)
                {
                    sequences = Permutations(subset);
                }
                else
                {
                    // size >= 3 時 permutation 很容易爆炸。
                    // 這裡保留幾種常見排序，避免 candidate 數量太大。
                    sequences = GenerateLimitedSequences(subset);
                }

                HashSet<string> subsetCommonMachines = GetCommonFeasibleMachines(data, subset);
                if (subsetCommonMachines.Count == 0) continue;

                foreach (List<Order> sequence in sequences)
                {
                    foreach (string machineNo in subsetCommonMachines)
                    {
                        if (!data.Machines.ContainsKey(machineNo)) continue;

                        Machine machine = data.Machines[machineNo];

                        if (!BasicGroupFeasible(data, sequence, machine)) continue;

                        CandidateGroup cg = BuildCandidateGroup(data, sequence, machineNo, tempId);
                        if (cg == null) continue;

                        if (!EarliestDueDatePossible(data, cg)) continue;

                        candidates.Add(cg);
                        tempId++;
                    }
                }
            }
        }

        candidates = candidates
            .OrderByDescending(c => c.OrderNos.Count)
            .ThenBy(c => c.MaxDueDate)
            .ThenBy(c => c.TotalSpanSeconds)
            .ThenBy(c => c.MachineNo)
            .ToList();

        if (candidates.Count > maxCandidates)
        {
            Console.WriteLine("Candidate count " + candidates.Count + " exceeds limit " + maxCandidates + ". Truncating candidates.");
            candidates = candidates.Take(maxCandidates).ToList();
        }

        for (int i = 0; i < candidates.Count; i++)
        {
            candidates[i].CandidateId = i + 1;
        }

        return candidates;
    }

    static void PrintCandidateSummary(List<CandidateGroup> candidates)
    {
        var groups = candidates
            .GroupBy(c => c.OrderNos.Count)
            .OrderBy(g => g.Key);

        foreach (var g in groups)
        {
            Console.WriteLine("  Size " + g.Key + ": " + g.Count());
        }
    }

    static List<List<Order>> GenerateLimitedSequences(List<Order> subset)
    {
        List<List<Order>> sequences = new List<List<Order>>();

        sequences.Add(subset.OrderBy(o => o.DueDate).ThenBy(o => o.OrderNo).ToList());
        sequences.Add(subset.OrderBy(o => o.ProcessTimeSeconds).ThenBy(o => o.DueDate).ToList());
        sequences.Add(subset.OrderBy(o => o.Type).ThenBy(o => o.MaterialNo).ThenBy(o => o.DueDate).ToList());

        // 去除重複排序。
        Dictionary<string, List<Order>> unique = new Dictionary<string, List<Order>>();

        foreach (List<Order> seq in sequences)
        {
            string key = string.Join("-", seq.Select(o => o.OrderNo));
            unique[key] = seq;
        }

        return unique.Values.ToList();
    }

    static HashSet<string> GetCommonFeasibleMachines(InputData data, List<Order> sequence)
    {
        HashSet<string> common = null;

        foreach (Order o in sequence)
        {
            if (!data.RoutesByMaterial.ContainsKey(o.MaterialNo))
            {
                return new HashSet<string>();
            }

            HashSet<string> machines = data.RoutesByMaterial[o.MaterialNo]
                .Select(r => r.MachineNo)
                .Where(m => data.Machines.ContainsKey(m))
                .ToHashSet();

            if (common == null)
            {
                common = machines;
            }
            else
            {
                common.IntersectWith(machines);
            }
        }

        return common ?? new HashSet<string>();
    }

    static bool BasicGroupFeasible(InputData data, List<Order> sequence, Machine machine)
    {
        int totalMoldRequired = 0;
        HashSet<string> moldTypes = new HashSet<string>();

        foreach (Order o in sequence)
        {
            if (o.Qty > machine.Capacity) return false;

            Route route = GetRoute(data, o.MaterialNo, machine.MachineNo);
            if (route == null) return false;

            if (moldTypes.Contains(route.MoldNo)) return false;

            moldTypes.Add(route.MoldNo);
            totalMoldRequired += o.MoldRequiredQty;
        }

        if (totalMoldRequired > machine.Cavity) return false;

        return true;
    }

    static CandidateGroup BuildCandidateGroup(InputData data, List<Order> sequence, string machineNo, int id)
    {
        CandidateGroup cg = new CandidateGroup();
        cg.CandidateId = id;
        cg.MachineNo = machineNo;
        cg.OrderNos = sequence.Select(o => o.OrderNo).ToList();
        cg.OrderSegments = new Dictionary<int, TimeSegment>();
        cg.MoldUsage = new Dictionary<string, int>();

        int groupSize = sequence.Count;
        double current = 0.0;

        for (int i = 0; i < sequence.Count; i++)
        {
            Order o = sequence[i];
            Route route = GetRoute(data, o.MaterialNo, machineNo);

            if (route == null) return null;

            if (!cg.MoldUsage.ContainsKey(route.MoldNo))
            {
                cg.MoldUsage[route.MoldNo] = 0;
            }

            cg.MoldUsage[route.MoldNo] += o.MoldRequiredQty;

            double adjustedProcessSeconds = o.ProcessTimeSeconds * groupSize;
            double startOffset = current;
            double endOffset = startOffset + adjustedProcessSeconds;

            cg.OrderSegments[o.OrderNo] = new TimeSegment
            {
                StartOffsetSeconds = startOffset,
                EndOffsetSeconds = endOffset
            };

            current = endOffset;

            if (i < sequence.Count - 1)
            {
                current += GetInternalSetupSeconds(o, sequence[i + 1]);
            }
        }

        cg.TotalSpanSeconds = current;
        cg.MinDueDate = sequence.Min(o => o.DueDate);
        cg.MaxDueDate = sequence.Max(o => o.DueDate);

        return cg;
    }

    static bool EarliestDueDatePossible(InputData data, CandidateGroup cg)
    {
        DateTime earliestStart = BaseTime.AddSeconds(FirstPreSetupSeconds);

        foreach (int orderNo in cg.OrderNos)
        {
            Order o = data.Orders[orderNo];
            DateTime orderEnd = earliestStart.AddSeconds(cg.OrderSegments[orderNo].EndOffsetSeconds);

            if (orderEnd > o.DueDate)
            {
                return false;
            }
        }

        return true;
    }

    static Route GetRoute(InputData data, string materialNo, string machineNo)
    {
        if (!data.RoutesByMaterial.ContainsKey(materialNo)) return null;

        return data.RoutesByMaterial[materialNo]
            .FirstOrDefault(r => r.MachineNo == machineNo);
    }

    static double GetInternalSetupSeconds(Order prev, Order next)
    {
        if (prev.Type != next.Type)
        {
            return (CleanMinutes + TuneMinutes) * 60.0;
        }

        if (prev.MaterialNo == next.MaterialNo)
        {
            return 0.0;
        }

        return TuneMinutes * 60.0;
    }

    static List<List<Order>> Combinations(List<Order> orders, int size)
    {
        List<List<Order>> result = new List<List<Order>>();
        BuildCombination(orders, size, 0, new List<Order>(), result);
        return result;
    }

    static void BuildCombination(List<Order> orders, int size, int index, List<Order> current, List<List<Order>> result)
    {
        if (current.Count == size)
        {
            result.Add(new List<Order>(current));
            return;
        }

        for (int i = index; i < orders.Count; i++)
        {
            current.Add(orders[i]);
            BuildCombination(orders, size, i + 1, current, result);
            current.RemoveAt(current.Count - 1);
        }
    }

    static List<List<Order>> Permutations(List<Order> items)
    {
        List<List<Order>> result = new List<List<Order>>();
        BuildPermutation(items, new bool[items.Count], new List<Order>(), result);
        return result;
    }

    static void BuildPermutation(List<Order> items, bool[] used, List<Order> current, List<List<Order>> result)
    {
        if (current.Count == items.Count)
        {
            result.Add(new List<Order>(current));
            return;
        }

        for (int i = 0; i < items.Count; i++)
        {
            if (used[i]) continue;

            used[i] = true;
            current.Add(items[i]);

            BuildPermutation(items, used, current, result);

            current.RemoveAt(current.Count - 1);
            used[i] = false;
        }
    }

    // ============================================================
    // Output
    // ============================================================

    static List<ResultRow> BuildResultRows(InputData data, SolveResult solveResult)
    {
        List<ResultRow> rows = new List<ResultRow>();
        int outputGroupId = 1;

        foreach (ScheduledGroup sg in solveResult.Groups
            .OrderBy(g => g.MachineNo)
            .ThenBy(g => g.ProcessingStart))
        {
            CandidateGroup cg = sg.Candidate;
            int isPartial = cg.OrderNos.Count >= 2 ? 1 : 0;

            foreach (int orderNo in cg.OrderNos)
            {
                Order o = data.Orders[orderNo];
                TimeSegment seg = cg.OrderSegments[orderNo];

                rows.Add(new ResultRow
                {
                    OrderNo = orderNo,
                    GroupId = outputGroupId.ToString(),
                    IsPartialGroup = isPartial.ToString(),
                    MachineNo = sg.MachineNo,
                    StartTime = FormatTime(sg.ProcessingStart.AddSeconds(seg.StartOffsetSeconds)),
                    EndTime = FormatTime(sg.ProcessingStart.AddSeconds(seg.EndOffsetSeconds)),
                    DueTime = FormatTime(o.DueDate),
                    IsScheduled = 1
                });
            }

            outputGroupId++;
        }

        HashSet<int> scheduledOrders = rows
            .Where(r => r.IsScheduled == 1)
            .Select(r => r.OrderNo)
            .ToHashSet();

        foreach (Order o in data.Orders.Values.OrderBy(o => o.OrderNo))
        {
            if (scheduledOrders.Contains(o.OrderNo)) continue;

            rows.Add(new ResultRow
            {
                OrderNo = o.OrderNo,
                GroupId = "",
                IsPartialGroup = "",
                MachineNo = "",
                StartTime = "",
                EndTime = "",
                DueTime = FormatTime(o.DueDate),
                IsScheduled = 0
            });
        }

        return rows.OrderBy(r => r.OrderNo).ToList();
    }

    static void WriteResultCsv(string path, List<ResultRow> rows, double runtimeSeconds, int objValue)
    {
        using (StreamWriter sw = new StreamWriter(path))
        {
            sw.WriteLine("Time," + runtimeSeconds.ToString("0.000", CultureInfo.InvariantCulture) + ",ObjValue," + objValue);
            sw.WriteLine("ORDER_NO,GROUP_ID,IS_PARTIAL_GROUP,MACHINE_NO,START_TIME,END_TIME,DUE_TIME,IS_SCHEDULED");

            foreach (ResultRow r in rows)
            {
                sw.WriteLine(string.Join(",", new string[]
                {
                    r.OrderNo.ToString(),
                    r.GroupId,
                    r.IsPartialGroup,
                    r.MachineNo,
                    r.StartTime,
                    r.EndTime,
                    r.DueTime,
                    r.IsScheduled.ToString()
                }));
            }
        }
    }

    // ============================================================
    // Validation
    // ============================================================

    static List<string> ValidateSolution(InputData data, SolveResult solveResult, List<ResultRow> rows)
    {
        List<string> report = new List<string>();
        bool ok = true;

        report.Add("Validation Report");
        report.Add("=================");
        report.Add("");
        report.Add("CPLEX solved : " + solveResult.CplexSolved);
        report.Add("CPLEX status : " + solveResult.CplexStatus);
        report.Add("Model obj    : " + solveResult.ModelObjectiveValue.ToString("0.###", CultureInfo.InvariantCulture));
        report.Add("Best bound   : " + solveResult.BestBound.ToString("0.###", CultureInfo.InvariantCulture));
        report.Add("");

        int scheduledCount = rows.Count(r => r.IsScheduled == 1);
        report.Add("Scheduled orders: " + scheduledCount + " / " + data.Orders.Count);

        // 1. Each scheduled order appears once.
        var duplicated = rows
            .Where(r => r.IsScheduled == 1)
            .GroupBy(r => r.OrderNo)
            .Where(g => g.Count() > 1)
            .Select(g => g.Key)
            .ToList();

        if (duplicated.Count > 0)
        {
            ok = false;
            report.Add("[FAIL] Duplicate scheduled orders: " + string.Join(", ", duplicated));
        }
        else
        {
            report.Add("[OK] Each scheduled order appears once.");
        }

        // 2. Due date.
        foreach (ResultRow r in rows.Where(r => r.IsScheduled == 1))
        {
            DateTime end = ParseTime(r.EndTime);
            DateTime due = data.Orders[r.OrderNo].DueDate;

            if (end > due)
            {
                ok = false;
                report.Add("[FAIL] Due date violation: order " + r.OrderNo);
            }
        }

        report.Add("[OK] Due date check finished.");

        // 3. Route and capacity.
        foreach (ResultRow r in rows.Where(r => r.IsScheduled == 1))
        {
            Order o = data.Orders[r.OrderNo];

            if (!data.Machines.ContainsKey(r.MachineNo))
            {
                ok = false;
                report.Add("[FAIL] Unknown machine: " + r.MachineNo);
                continue;
            }

            Route route = GetRoute(data, o.MaterialNo, r.MachineNo);
            if (route == null)
            {
                ok = false;
                report.Add("[FAIL] Route violation: order " + r.OrderNo + " on " + r.MachineNo);
            }

            Machine m = data.Machines[r.MachineNo];
            if (o.Qty > m.Capacity)
            {
                ok = false;
                report.Add("[FAIL] Capacity violation: order " + r.OrderNo + " on " + r.MachineNo);
            }
        }

        report.Add("[OK] Route and capacity check finished.");

        // 4. Machine mold change limit.
        var groupByMachine = solveResult.Groups
            .GroupBy(g => g.MachineNo)
            .ToDictionary(g => g.Key, g => g.Count());

        foreach (Machine m in data.Machines.Values)
        {
            int used = groupByMachine.ContainsKey(m.MachineNo) ? groupByMachine[m.MachineNo] : 0;

            if (used > m.MoldChangeLimit)
            {
                ok = false;
                report.Add("[FAIL] Mold change limit violation: " + m.MachineNo + ", used = " + used + ", limit = " + m.MoldChangeLimit);
            }
        }

        report.Add("[OK] Mold change limit check finished.");

        // 5. Cavity and same mold type in one partial group.
        foreach (ScheduledGroup sg in solveResult.Groups)
        {
            CandidateGroup cg = sg.Candidate;
            Machine m = data.Machines[sg.MachineNo];

            int totalMoldRequired = 0;
            HashSet<string> moldTypes = new HashSet<string>();

            foreach (int orderNo in cg.OrderNos)
            {
                Order o = data.Orders[orderNo];
                Route route = GetRoute(data, o.MaterialNo, sg.MachineNo);

                if (route == null)
                {
                    ok = false;
                    report.Add("[FAIL] Route missing inside group: order " + orderNo);
                    continue;
                }

                if (moldTypes.Contains(route.MoldNo))
                {
                    ok = false;
                    report.Add("[FAIL] Same mold type in one partial group: " + route.MoldNo + " group candidate " + cg.CandidateId);
                }

                moldTypes.Add(route.MoldNo);
                totalMoldRequired += o.MoldRequiredQty;
            }

            if (totalMoldRequired > m.Cavity)
            {
                ok = false;
                report.Add("[FAIL] Cavity violation on " + sg.MachineNo + ", required = " + totalMoldRequired + ", cavity = " + m.Cavity);
            }
        }

        report.Add("[OK] Group cavity and mold-type check finished.");

        // 6. Machine sequencing and maintenance.
        foreach (string machineNo in data.Machines.Keys)
        {
            List<ScheduledGroup> machineGroups = solveResult.Groups
                .Where(g => g.MachineNo == machineNo)
                .OrderBy(g => g.ProcessingStart)
                .ToList();

            List<Maintenance> maints = data.MaintenancesByMachine.ContainsKey(machineNo)
                ? data.MaintenancesByMachine[machineNo]
                : new List<Maintenance>();

            for (int i = 0; i < machineGroups.Count; i++)
            {
                ScheduledGroup g = machineGroups[i];

                // Processing cannot overlap maintenance.
                foreach (Maintenance mt in maints)
                {
                    if (Overlap(g.ProcessingStart, g.ProcessingEnd, mt.StartTime, mt.EndTime))
                    {
                        ok = false;
                        report.Add("[FAIL] Processing overlaps maintenance: " + machineNo + " candidate " + g.Candidate.CandidateId);
                    }
                }

                // First group setup: Mount + Tune before processing.
                if (i == 0)
                {
                    DateTime setupStart = g.ProcessingStart.AddSeconds(-FirstPreSetupSeconds);
                    DateTime setupEnd = g.ProcessingStart;

                    if (setupStart < BaseTime)
                    {
                        ok = false;
                        report.Add("[FAIL] First setup starts before base time: " + machineNo);
                    }

                    foreach (Maintenance mt in maints)
                    {
                        if (Overlap(setupStart, setupEnd, mt.StartTime, mt.EndTime))
                        {
                            ok = false;
                            report.Add("[FAIL] First setup overlaps maintenance: " + machineNo);
                        }
                    }
                }
                else
                {
                    ScheduledGroup prev = machineGroups[i - 1];

                    if (prev.ProcessingEnd > g.ProcessingStart)
                    {
                        ok = false;
                        report.Add("[FAIL] Machine processing overlap: " + machineNo);
                    }

                    if (!GapCanContainBetweenGroupSetup(prev.ProcessingEnd, g.ProcessingStart, maints))
                    {
                        ok = false;
                        report.Add("[FAIL] Not enough setup gap between groups on " + machineNo);
                    }
                }

                // Last unmount check.
                if (i == machineGroups.Count - 1)
                {
                    DateTime unmountStart = g.ProcessingEnd;
                    DateTime unmountEnd = g.ProcessingEnd.AddSeconds(LastPostSetupSeconds);

                    foreach (Maintenance mt in maints)
                    {
                        if (Overlap(unmountStart, unmountEnd, mt.StartTime, mt.EndTime))
                        {
                            ok = false;
                            report.Add("[FAIL] Last unmount overlaps maintenance: " + machineNo);
                        }
                    }
                }
            }
        }

        report.Add("[OK] Machine sequencing and maintenance check finished.");

        // 7. Mold availability over time.
        List<MoldInterval> moldIntervals = new List<MoldInterval>();

        foreach (ScheduledGroup sg in solveResult.Groups)
        {
            foreach (KeyValuePair<string, int> kv in sg.Candidate.MoldUsage)
            {
                moldIntervals.Add(new MoldInterval
                {
                    MoldNo = kv.Key,
                    Quantity = kv.Value,
                    StartTime = sg.ProcessingStart,
                    EndTime = sg.ProcessingEnd
                });
            }
        }

        foreach (string moldNo in data.Molds.Keys)
        {
            int capacity = data.Molds[moldNo].Qty;

            List<MoldEvent> events = new List<MoldEvent>();

            foreach (MoldInterval mi in moldIntervals.Where(x => x.MoldNo == moldNo))
            {
                events.Add(new MoldEvent { Time = mi.StartTime, Delta = mi.Quantity });
                events.Add(new MoldEvent { Time = mi.EndTime, Delta = -mi.Quantity });
            }

            events = events
                .OrderBy(e => e.Time)
                .ThenBy(e => e.Delta)
                .ToList();

            int used = 0;

            foreach (MoldEvent e in events)
            {
                used += e.Delta;

                if (used > capacity)
                {
                    ok = false;
                    report.Add("[FAIL] Mold availability violation: " + moldNo + ", used = " + used + ", available = " + capacity);
                    break;
                }
            }
        }

        report.Add("[OK] Mold availability check finished.");
        report.Add("");
        report.Add(ok ? "FINAL RESULT: FEASIBLE" : "FINAL RESULT: INFEASIBLE");

        return report;
    }

    static bool GapCanContainBetweenGroupSetup(DateTime prevEnd, DateTime nextStart, List<Maintenance> maints)
    {
        if (nextStart <= prevEnd) return false;

        List<Maintenance> between = maints
            .Where(mt => mt.EndTime > prevEnd && mt.StartTime < nextStart)
            .OrderBy(mt => mt.StartTime)
            .ToList();

        if (between.Count == 0)
        {
            return nextStart >= prevEnd.AddSeconds(BetweenSetupSeconds);
        }

        Maintenance first = between.First();
        Maintenance last = between.Last();

        bool beforeMaintenanceOK = prevEnd.AddSeconds(BetweenBeforeMaintenanceSeconds) <= first.StartTime;
        bool afterMaintenanceOK = last.EndTime.AddSeconds(BetweenAfterMaintenanceSeconds) <= nextStart;

        return beforeMaintenanceOK && afterMaintenanceOK;
    }

    static bool Overlap(DateTime aStart, DateTime aEnd, DateTime bStart, DateTime bEnd)
    {
        return aStart < bEnd && bStart < aEnd;
    }

    // ============================================================
    // Read input files
    // ============================================================

    static InputData ReadAllData(string folder)
    {
        InputData data = new InputData();

        string orderPath = FindFile(folder, "ORDER.csv");
        string machinePath = FindFile(folder, "MACHINE.csv");
        string moldPath = FindFile(folder, "MOLD.csv");
        string routePath = FindFile(folder, "MATERIAL_ROUTE.csv");
        string maintenancePath = FindFile(folder, "MAINTENANCE.csv");

        data.Orders = ReadOrders(orderPath);
        data.Machines = ReadMachines(machinePath);
        data.Molds = ReadMolds(moldPath);
        data.Routes = ReadRoutes(routePath);
        data.Maintenances = ReadMaintenances(maintenancePath);

        data.RoutesByMaterial = data.Routes
            .GroupBy(r => r.MaterialNo)
            .ToDictionary(g => g.Key, g => g.ToList());

        data.MaintenancesByMachine = data.Machines.Keys.ToDictionary(m => m, m => new List<Maintenance>());

        foreach (Maintenance mt in data.Maintenances)
        {
            if (!data.MaintenancesByMachine.ContainsKey(mt.MachineNo))
            {
                data.MaintenancesByMachine[mt.MachineNo] = new List<Maintenance>();
            }

            data.MaintenancesByMachine[mt.MachineNo].Add(mt);
        }

        foreach (string machineNo in data.MaintenancesByMachine.Keys.ToList())
        {
            data.MaintenancesByMachine[machineNo] = data.MaintenancesByMachine[machineNo]
                .OrderBy(mt => mt.StartTime)
                .ToList();
        }

        return data;
    }

    static string FindFile(string folder, string fileName)
    {
        string direct = Path.Combine(folder, fileName);

        if (File.Exists(direct))
        {
            return direct;
        }

        string nameWithoutExtension = Path.GetFileNameWithoutExtension(fileName);
        string[] matches = Directory.GetFiles(folder, nameWithoutExtension + "*.csv");

        if (matches.Length == 0)
        {
            throw new FileNotFoundException("Cannot find input file: " + fileName);
        }

        return matches[0];
    }

    static Dictionary<int, Order> ReadOrders(string path)
    {
        Dictionary<int, Order> orders = new Dictionary<int, Order>();
        string[] lines = File.ReadAllLines(path);

        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] p = SplitCsvLine(lines[i]);

            Order o = new Order();
            o.OrderNo = int.Parse(Clean(p[0]));
            o.MaterialNo = Clean(p[1]);
            o.Type = Clean(p[2]);
            o.DueDate = ParseTime(Clean(p[3]));
            o.Qty = int.Parse(Clean(p[4]));
            o.ProcessTimeSeconds = double.Parse(Clean(p[5]), CultureInfo.InvariantCulture);
            o.Unit = Clean(p[6]);
            o.MoldRequiredQty = int.Parse(Clean(p[7]));

            orders[o.OrderNo] = o;
        }

        return orders;
    }

    static Dictionary<string, Machine> ReadMachines(string path)
    {
        Dictionary<string, Machine> machines = new Dictionary<string, Machine>();
        string[] lines = File.ReadAllLines(path);

        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] p = SplitCsvLine(lines[i]);

            Machine m = new Machine();
            m.MachineNo = Clean(p[0]);
            m.Capacity = int.Parse(Clean(p[1]));
            m.Cavity = int.Parse(Clean(p[2]));
            m.MoldChangeLimit = int.Parse(Clean(p[3]));

            machines[m.MachineNo] = m;
        }

        return machines;
    }

    static Dictionary<string, Mold> ReadMolds(string path)
    {
        Dictionary<string, Mold> molds = new Dictionary<string, Mold>();
        string[] lines = File.ReadAllLines(path);

        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] p = SplitCsvLine(lines[i]);

            Mold q = new Mold();
            q.MoldNo = Clean(p[0]);
            q.Qty = int.Parse(Clean(p[1]));

            molds[q.MoldNo] = q;
        }

        return molds;
    }

    static List<Route> ReadRoutes(string path)
    {
        List<Route> routes = new List<Route>();
        string[] lines = File.ReadAllLines(path);

        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] p = SplitCsvLine(lines[i]);

            Route r = new Route();
            r.MaterialNo = Clean(p[0]);
            r.MachineNo = Clean(p[1]);
            r.MoldNo = Clean(p[2]);

            routes.Add(r);
        }

        return routes;
    }

    static List<Maintenance> ReadMaintenances(string path)
    {
        List<Maintenance> maintenances = new List<Maintenance>();
        string[] lines = File.ReadAllLines(path);

        for (int i = 1; i < lines.Length; i++)
        {
            if (string.IsNullOrWhiteSpace(lines[i])) continue;

            string[] p = SplitCsvLine(lines[i]);

            Maintenance mt = new Maintenance();
            mt.MachineNo = Clean(p[0]);
            mt.StartTime = ParseTime(Clean(p[1]));
            mt.EndTime = ParseTime(Clean(p[2]));

            maintenances.Add(mt);
        }

        return maintenances;
    }

    static string[] SplitCsvLine(string line)
    {
        return line.Split(',');
    }

    static string Clean(string s)
    {
        return s.Trim().Trim('\uFEFF');
    }

    static DateTime ParseTime(string s)
    {
        return DateTime.ParseExact(s, "yyyy/MM/dd HH:mm:ss", CultureInfo.InvariantCulture);
    }

    static string FormatTime(DateTime dt)
    {
        return dt.ToString("yyyy/MM/dd HH:mm:ss");
    }
}

// ============================================================
// Data classes
// ============================================================

class InputData
{
    public Dictionary<int, Order> Orders = new Dictionary<int, Order>();
    public Dictionary<string, Machine> Machines = new Dictionary<string, Machine>();
    public Dictionary<string, Mold> Molds = new Dictionary<string, Mold>();
    public List<Route> Routes = new List<Route>();
    public List<Maintenance> Maintenances = new List<Maintenance>();

    public Dictionary<string, List<Route>> RoutesByMaterial = new Dictionary<string, List<Route>>();
    public Dictionary<string, List<Maintenance>> MaintenancesByMachine = new Dictionary<string, List<Maintenance>>();
}

class Order
{
    public int OrderNo;
    public string MaterialNo = "";
    public string Type = "";
    public DateTime DueDate;
    public int Qty;
    public double ProcessTimeSeconds;
    public string Unit = "";
    public int MoldRequiredQty;
}

class Machine
{
    public string MachineNo = "";
    public int Capacity;
    public int Cavity;
    public int MoldChangeLimit;
}

class Mold
{
    public string MoldNo = "";
    public int Qty;
}

class Route
{
    public string MaterialNo = "";
    public string MachineNo = "";
    public string MoldNo = "";
}

class Maintenance
{
    public string MachineNo = "";
    public DateTime StartTime;
    public DateTime EndTime;
}

class CandidateGroup
{
    public int CandidateId;
    public string MachineNo = "";
    public List<int> OrderNos = new List<int>();
    public Dictionary<int, TimeSegment> OrderSegments = new Dictionary<int, TimeSegment>();
    public Dictionary<string, int> MoldUsage = new Dictionary<string, int>();
    public double TotalSpanSeconds;
    public DateTime MinDueDate;
    public DateTime MaxDueDate;
}

class TimeSegment
{
    public double StartOffsetSeconds;
    public double EndOffsetSeconds;
}

class ScheduledGroup
{
    public CandidateGroup Candidate = null;
    public string MachineNo = "";
    public DateTime ProcessingStart;
    public DateTime ProcessingEnd;
}

class SolveResult
{
    public bool CplexSolved = false;
    public string CplexStatus = "";
    public double ModelObjectiveValue = 0.0;
    public double BestBound = 0.0;
    public List<ScheduledGroup> Groups = new List<ScheduledGroup>();
}

class ResultRow
{
    public int OrderNo;
    public string GroupId = "";
    public string IsPartialGroup = "";
    public string MachineNo = "";
    public string StartTime = "";
    public string EndTime = "";
    public string DueTime = "";
    public int IsScheduled;
}

class MoldInterval
{
    public string MoldNo = "";
    public int Quantity;
    public DateTime StartTime;
    public DateTime EndTime;
}

class MoldEvent
{
    public DateTime Time;
    public int Delta;
}
