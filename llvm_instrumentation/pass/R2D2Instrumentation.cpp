#include "llvm/ADT/StringRef.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Passes/PassPlugin.h"

#include <array>
#include <cstdint>
#include <optional>

using namespace llvm;

namespace {

enum class Event : std::uint32_t {
  RclNodeInit = 0,
  RclSubscriptionInit = 1,
  RclServiceInit = 2,
  RclcppSubscriptionInit = 3,
  RclcppSubscriptionCallbackAdded = 4,
  RclcppServiceCallbackAdded = 5,
  RclcppTimerCallbackAdded = 6,
  RclcppTimerLinkNode = 7,
  ExecutorExecute = 8,
  CallbackStart = 9,
  CallbackEnd = 10,
  RclTake = 11,
};

struct Site {
  CallBase* call;
  Event event;
  std::array<int, 4> arguments;
};

std::optional<std::pair<Event, std::array<int, 4>>> classify(StringRef name) {
  // arguments are {pointer0, pointer1, string0, string1}; -1 means null.
  // Values <= -2 address the enclosing function arguments: -2 is function
  // argument 0, -3 is function argument 1, and so on. This is needed for
  // ros_trace_rcl_take(), whose official tracepoint only carries the message
  // pointer; the rcl subscription handle is the first argument of rcl_take*().
  if (name == "ros_trace_rcl_node_init") {
    return {{Event::RclNodeInit, {0, -1, 2, 3}}};
  }
  if (name == "ros_trace_rcl_subscription_init") {
    return {{Event::RclSubscriptionInit, {0, 1, 3, -1}}};
  }
  if (name == "ros_trace_rcl_service_init") {
    return {{Event::RclServiceInit, {0, 1, 3, -1}}};
  }
  if (name == "ros_trace_rclcpp_subscription_init") {
    return {{Event::RclcppSubscriptionInit, {0, 1, -1, -1}}};
  }
  if (name == "ros_trace_rclcpp_subscription_callback_added") {
    return {{Event::RclcppSubscriptionCallbackAdded, {0, 1, -1, -1}}};
  }
  if (name == "ros_trace_rclcpp_service_callback_added") {
    return {{Event::RclcppServiceCallbackAdded, {0, 1, -1, -1}}};
  }
  if (name == "ros_trace_rclcpp_timer_callback_added") {
    return {{Event::RclcppTimerCallbackAdded, {0, 1, -1, -1}}};
  }
  if (name == "ros_trace_rclcpp_timer_link_node") {
    return {{Event::RclcppTimerLinkNode, {0, 1, -1, -1}}};
  }
  if (name == "ros_trace_rclcpp_executor_execute") {
    return {{Event::ExecutorExecute, {0, -1, -1, -1}}};
  }
  if (name == "ros_trace_callback_start") {
    return {{Event::CallbackStart, {0, -1, -1, -1}}};
  }
  if (name == "ros_trace_callback_end") {
    return {{Event::CallbackEnd, {0, -1, -1, -1}}};
  }
  if (name == "ros_trace_rcl_take") {
    return {{Event::RclTake, {-2, 0, -1, -1}}};
  }
  return std::nullopt;
}

Value* pointerArgument(IRBuilder<>& builder, CallBase& call, int index) {
  PointerType* pointer = PointerType::getUnqual(builder.getContext());
  if (index == -1) {
    return ConstantPointerNull::get(pointer);
  }
  if (index <= -2) {
    const unsigned function_arg_index = static_cast<unsigned>(-index - 2);
    Function* function = call.getFunction();
    if (function == nullptr || function_arg_index >= function->arg_size()) {
      return ConstantPointerNull::get(pointer);
    }
    Value* value = function->getArg(function_arg_index);
    if (value->getType()->isPointerTy()) {
      return builder.CreatePointerCast(value, pointer);
    }
    return ConstantPointerNull::get(pointer);
  }
  if (static_cast<unsigned>(index) >= call.arg_size()) {
    return ConstantPointerNull::get(pointer);
  }
  Value* value = call.getArgOperand(static_cast<unsigned>(index));
  if (value->getType()->isPointerTy()) {
    return builder.CreatePointerCast(value, pointer);
  }
  return ConstantPointerNull::get(pointer);
}

class R2D2InstrumentationPass
    : public PassInfoMixin<R2D2InstrumentationPass> {
 public:
  PreservedAnalyses run(Module& module, ModuleAnalysisManager&) {
    SmallVector<Site, 64> sites;
    for (Function& function : module) {
      if (function.isDeclaration()) {
        continue;
      }
      for (BasicBlock& block : function) {
        for (Instruction& instruction : block) {
          auto* call = dyn_cast<CallBase>(&instruction);
          if (call == nullptr) {
            continue;
          }
          if (call->getMetadata("r2d2.instrumented") != nullptr) {
            continue;
          }
          Value* called = call->getCalledOperand()->stripPointerCasts();
          Function* target = dyn_cast<Function>(called);
          if (target == nullptr) {
            continue;
          }
          auto classified = classify(target->getName());
          if (classified.has_value()) {
            sites.push_back({call, classified->first, classified->second});
          }
        }
      }
    }

    if (sites.empty()) {
      return PreservedAnalyses::all();
    }

    LLVMContext& context = module.getContext();
    PointerType* pointer = PointerType::getUnqual(context);
    FunctionType* hook_type = FunctionType::get(
        Type::getVoidTy(context),
        {Type::getInt32Ty(context), pointer, pointer, pointer, pointer}, false);
    FunctionCallee hook = module.getOrInsertFunction("__r2d2_llvm_trace", hook_type);

    for (const Site& site : sites) {
      // Insert immediately before the standard ROS tracepoint. Registration
      // tracepoints in constructors are commonly represented as `invoke` in
      // LLVM IR, so relying on a post-call insertion misses those sites.
      IRBuilder<> builder(site.call);
      builder.CreateCall(
          hook,
          {builder.getInt32(static_cast<std::uint32_t>(site.event)),
           pointerArgument(builder, *site.call, site.arguments[0]),
           pointerArgument(builder, *site.call, site.arguments[1]),
           pointerArgument(builder, *site.call, site.arguments[2]),
           pointerArgument(builder, *site.call, site.arguments[3])});
      site.call->setMetadata("r2d2.instrumented", MDNode::get(context, {}));
    }

    module.addModuleFlag(Module::Warning, "r2d2.llvm.instrumented",
                         static_cast<std::uint32_t>(sites.size()));
    return PreservedAnalyses::none();
  }
};

}  // namespace

extern "C" LLVM_ATTRIBUTE_WEAK PassPluginLibraryInfo llvmGetPassPluginInfo() {
  return {LLVM_PLUGIN_API_VERSION, "R2D2Instrumentation", LLVM_VERSION_STRING,
          [](PassBuilder& builder) {
            builder.registerPipelineParsingCallback(
                [](StringRef name, ModulePassManager& manager,
                   ArrayRef<PassBuilder::PipelineElement>) {
                  if (name != "r2d2-instrument") {
                    return false;
                  }
                  manager.addPass(R2D2InstrumentationPass());
                  return true;
                });
            builder.registerOptimizerLastEPCallback(
                [](ModulePassManager& manager, OptimizationLevel) {
                  manager.addPass(R2D2InstrumentationPass());
                });
          }};
}
