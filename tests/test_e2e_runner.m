function exit_code = test_e2e_runner()
    % TEST_E2E_RUNNER Master End-to-End Test Suite Runner for myPalletizer 260 Pi
    % Executes Tiers 1-4, aggregates results, exports test reports, and returns exit code.
    %
    % Usage in MATLAB:
    %   exit_code = test_e2e_runner();
    %
    % Usage in Command Line / CI:
    %   matlab -batch "addpath('d:\Lidar\myPalletizer_BTL\tests'); exit(test_e2e_runner());"

    t_suite_start = tic;
    test_utils.add_project_paths();
    base_dir = test_utils.get_base_dir();

    fprintf('\n');
    fprintf('################################################################################\n');
    fprintf('#                                                                              #\n');
    fprintf('#   4-DOF myPalletizer 260 Pi Simulation System - Master E2E Test Runner       #\n');
    fprintf('#   MATLAB R2025a | Strictly Pure Native MATLAB | Zero Peter Corke Toolbox     #\n');
    fprintf('#                                                                              #\n');
    fprintf('################################################################################\n\n');

    % Execute All 4 Tiers
    t1_results = tier1_feature_tests();
    t2_results = tier2_boundary_tests();
    t3_results = tier3_pairwise_tests();
    t4_results = tier4_workload_tests();

    all_results = [t1_results, t2_results, t3_results, t4_results];
    total_duration_sec = toc(t_suite_start);

    % Compute Statistics
    stats.t1_total = numel(t1_results);
    stats.t1_pass = sum(strcmp({t1_results.status}, 'PASS'));
    stats.t1_fail = sum(strcmp({t1_results.status}, 'FAIL'));
    stats.t1_pending = sum(strcmp({t1_results.status}, 'PENDING'));

    stats.t2_total = numel(t2_results);
    stats.t2_pass = sum(strcmp({t2_results.status}, 'PASS'));
    stats.t2_fail = sum(strcmp({t2_results.status}, 'FAIL'));
    stats.t2_pending = sum(strcmp({t2_results.status}, 'PENDING'));

    stats.t3_total = numel(t3_results);
    stats.t3_pass = sum(strcmp({t3_results.status}, 'PASS'));
    stats.t3_fail = sum(strcmp({t3_results.status}, 'FAIL'));
    stats.t3_pending = sum(strcmp({t3_results.status}, 'PENDING'));

    stats.t4_total = numel(t4_results);
    stats.t4_pass = sum(strcmp({t4_results.status}, 'PASS'));
    stats.t4_fail = sum(strcmp({t4_results.status}, 'FAIL'));
    stats.t4_pending = sum(strcmp({t4_results.status}, 'PENDING'));

    stats.grand_total = numel(all_results);
    stats.grand_pass = sum(strcmp({all_results.status}, 'PASS'));
    stats.grand_fail = sum(strcmp({all_results.status}, 'FAIL'));
    stats.grand_pending = sum(strcmp({all_results.status}, 'PENDING'));
    stats.pass_rate_pct = (stats.grand_pass / (stats.grand_total - stats.grand_pending)) * 100.0;

    % Print Grand Master Summary Table
    fprintf('\n');
    fprintf('================================================================================\n');
    fprintf('                       GRAND E2E TEST EXECUTION SUMMARY                         \n');
    fprintf('================================================================================\n');
    fprintf('  %-22s | %-8s | %-8s | %-8s | %-8s | %-9s\n', ...
        'Test Tier', 'Total', 'PASS', 'FAIL', 'PENDING', 'Pass Rate');
    fprintf('--------------------------------------------------------------------------------\n');
    fprintf('  %-22s | %-8d | %-8d | %-8d | %-8d | %6.1f%%\n', ...
        'Tier 1: Feature Tests', stats.t1_total, stats.t1_pass, stats.t1_fail, stats.t1_pending, ...
        (stats.t1_pass / max(1, stats.t1_total - stats.t1_pending)) * 100);
    fprintf('  %-22s | %-8d | %-8d | %-8d | %-8d | %6.1f%%\n', ...
        'Tier 2: Boundary Tests', stats.t2_total, stats.t2_pass, stats.t2_fail, stats.t2_pending, ...
        (stats.t2_pass / max(1, stats.t2_total - stats.t2_pending)) * 100);
    fprintf('  %-22s | %-8d | %-8d | %-8d | %-8d | %6.1f%%\n', ...
        'Tier 3: Pairwise Tests', stats.t3_total, stats.t3_pass, stats.t3_fail, stats.t3_pending, ...
        (stats.t3_pass / max(1, stats.t3_total - stats.t3_pending)) * 100);
    fprintf('  %-22s | %-8d | %-8d | %-8d | %-8d | %6.1f%%\n', ...
        'Tier 4: Workload Tests', stats.t4_total, stats.t4_pass, stats.t4_fail, stats.t4_pending, ...
        (stats.t4_pass / max(1, stats.t4_total - stats.t4_pending)) * 100);
    fprintf('================================================================================\n');
    fprintf('  %-22s | %-8d | %-8d | %-8d | %-8d | %6.1f%%\n', ...
        'GRAND TOTAL SUITE', stats.grand_total, stats.grand_pass, stats.grand_fail, stats.grand_pending, ...
        stats.pass_rate_pct);
    fprintf('================================================================================\n');
    fprintf('  Total Execution Time : %.2f seconds\n', total_duration_sec);

    % Export Machine-Readable JSON Report
    report_file_json = fullfile(base_dir, 'tests', 'test_results.json');
    report_file_mat  = fullfile(base_dir, 'tests', 'test_results.mat');
    
    report_struct.timestamp = char(datetime('now', 'TimeZone', 'Z', 'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));
    report_struct.statistics = stats;
    report_struct.test_details = all_results;
    
    try
        json_text = jsonencode(report_struct);
        fid = fopen(report_file_json, 'w');
        if fid >= 0
            fprintf(fid, '%s', json_text);
            fclose(fid);
            fprintf('  Machine-Readable Report: %s\n', report_file_json);
        end
    catch ME
        fprintf('  [Warning] Failed to write JSON report: %s\n', ME.message);
    end

    try
        save(report_file_mat, 'report_struct', 'all_results', 'stats');
        fprintf('  MATLAB Artifact Report : %s\n', report_file_mat);
    catch ME
        fprintf('  [Warning] Failed to write MAT report: %s\n', ME.message);
    end

    % Exit Code Determination
    if stats.grand_fail == 0
        fprintf('\n>>> SUITE STATUS: ALL TESTS PASSED SUCCESSFULLY (0 FAILURES) <<<\n\n');
        exit_code = 0;
    else
        fprintf('\n>>> SUITE STATUS: FAILURES DETECTED (%d TESTS FAILED) <<<\n\n', stats.grand_fail);
        exit_code = 1;
    end
end
