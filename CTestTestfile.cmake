# CMake generated Testfile for 
# Source directory: /Users/tianyangzhao/Codes/MIPSolvers
# Build directory: /Users/tianyangzhao/Codes/MIPSolvers
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[test_engine_api]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_engine_api")
set_tests_properties([=[test_engine_api]=] PROPERTIES  LABELS "unit" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;233;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;249;mipsolvers_add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
add_test([=[test_lp_solver]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_lp_solver")
set_tests_properties([=[test_lp_solver]=] PROPERTIES  LABELS "unit" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;233;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;250;mipsolvers_add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
add_test([=[test_milp_solver]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_milp_solver")
set_tests_properties([=[test_milp_solver]=] PROPERTIES  LABELS "unit" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;233;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;251;mipsolvers_add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
add_test([=[test_ipm_solver]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_ipm_solver")
set_tests_properties([=[test_ipm_solver]=] PROPERTIES  LABELS "unit" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;233;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;252;mipsolvers_add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
add_test([=[test_dual_simplex]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_dual_simplex")
set_tests_properties([=[test_dual_simplex]=] PROPERTIES  LABELS "unit" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;233;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;253;mipsolvers_add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
add_test([=[test_problem_validation]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_problem_validation")
set_tests_properties([=[test_problem_validation]=] PROPERTIES  LABELS "unit" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;233;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;255;mipsolvers_add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
add_test([=[test_adapter_registry]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_adapter_registry")
set_tests_properties([=[test_adapter_registry]=] PROPERTIES  LABELS "unit" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;233;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;256;mipsolvers_add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
add_test([=[test_presolve]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_presolve")
set_tests_properties([=[test_presolve]=] PROPERTIES  LABELS "unit" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;233;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;257;mipsolvers_add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
add_test([=[test_scuc_module]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_scuc_module")
set_tests_properties([=[test_scuc_module]=] PROPERTIES  LABELS "integration" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;269;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
add_test([=[test_market_simulation]=] "/Users/tianyangzhao/Codes/MIPSolvers/test_market_simulation")
set_tests_properties([=[test_market_simulation]=] PROPERTIES  LABELS "benchmark" TIMEOUT "600" _BACKTRACE_TRIPLES "/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;289;add_test;/Users/tianyangzhao/Codes/MIPSolvers/CMakeLists.txt;0;")
subdirs("_deps/embedded_highs")
subdirs("_deps/embedded_scip")
subdirs("_deps/catch2-build")
subdirs("src/engine/kernel/linear_algebra/highs_factor")
