set(SRC_FILES
    AutoDiffUtils.hpp
    autodiff.h
    autodiff.cpp
    # PositionMapEvaluator.hpp
    # PositionMapEvaluator.cpp
    AutoDiffRAII.hpp
    AutoDiffRAII.cpp
    amips.hpp
    amips.cpp

    FunctionEvaluator.hpp
    FunctionEvaluator.cpp
    DifferentiableFunctionEvaluator.hpp
    DifferentiableFunctionEvaluator.cpp
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})
