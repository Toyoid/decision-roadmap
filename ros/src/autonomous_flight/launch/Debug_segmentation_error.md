launch `run_fg_drm.launch` with:
```xml
<node pkg="autonomous_flight" type="fg_drm_node" name="fg_drm_node" output="screen" launch-prefix="gdb -ex run --args">
    <param name="robot_namespace" value="$(arg robot_namespace)"/>
</node>
```
After running the `fg_drm_node` for over 2h, the segmentation error is triggered again:
```bash
Thread 1 "fg_drm_node" received signal SIGSEGV, Segmentation fault.
0x00007ffff7992e4e in _int_malloc (av=av@entry=0x7ffff7ae7b80 <main_arena>, bytes=bytes@entry=64)
    at malloc.c:3679
3679	malloc.c: No such file or directory.
(gdb) 
```
Now 在 GDB 中输入 `bt full` 查看完整堆栈和局部变量：
```bash
(gdb) bt full
# 重点观察崩溃前代码中内存分配/释放的调用点
```
Here I get: 
```cpp
(gdb) bt full
#0  0x00007ffff7992e4e in _int_malloc (av=av@entry=0x7ffff7ae7b80 <main_arena>, 
    bytes=bytes@entry=64) at malloc.c:3679
        nb = <optimized out>
        idx = 5
        bin = 0x7ffff7ae7c20 <main_arena+160>
        victim = 0x7ffff7ae7c20 <main_arena+160>
        size = <optimized out>
        victim_index = <optimized out>
        remainder = <optimized out>
        remainder_size = <optimized out>
        block = <optimized out>
        bit = <optimized out>
        map = <optimized out>
        fwd = <optimized out>
        bck = <optimized out>
        tcache_unsorted_count = <optimized out>
        tcache_nb = <optimized out>
        tc_idx = <optimized out>
        return_cached = <optimized out>
        __PRETTY_FUNCTION__ = "_int_malloc"
#1  0x00007ffff7995299 in __GI___libc_malloc (bytes=64) at malloc.c:3066
        ar_ptr = 0x7ffff7ae7b80 <main_arena>
        victim = <optimized out>
        hook = <optimized out>
        tbytes = <optimized out>
        tc_idx = <optimized out>
        __PRETTY_FUNCTION__ = "__libc_malloc"
#2  0x00007ffff7bb4b29 in operator new(unsigned long) () from /lib/x86_64-linux-gnu/libstdc++.so.6
No symbol table info available.
#3  0x00007ffff737bc6c in void std::vector<std::shared_ptr<PRM::Node>, std::allocator<std::shared_ptr<PRM::Node> > >::_M_realloc_insert<std::shared_ptr<PRM::Node> const&>(__gnu_cxx::__normal_iterator<std::shared_ptr<PRM::Node>*, std::vector<std::shared_ptr<PRM::Node>, std::allocator<std::shared_ptr<PRM::Node> > > >, std::shared_ptr<PRM::Node> const&) ()
   from /home/lhx/coding-projects/decision-roadmap/ros/devel/lib/libglobal_planner.so
No symbol table info available.
#4  0x00007ffff737a832 in PRM::KDTree::nearestNeighbor(std::shared_ptr<PRM::Node>, std::shared_ptr<PRM::Node>, std::shared_ptr<PRM::Node>, int) ()
   from /home/lhx/coding-projects/decision-roadmap/ros/devel/lib/libglobal_planner.so
No symbol table info available.
#5  0x00007ffff737a505 in PRM::KDTree::nearestNeighbor(std::shared_ptr<PRM::Node>, std::shared_ptr<PRM::Node>, std::shared_ptr<PRM::Node>, int) ()
   from /home/lhx/coding-projects/decision-roadmap/ros/devel/lib/libglobal_planner.so
No symbol table info available.
.........
#31139 0x00007ffff737a505 in PRM::KDTree::nearestNeighbor(std::shared_ptr<PRM::Node>, std::shared_ptr<PRM::Node>, std::shared_ptr<PRM::Node>, int) ()
   from /home/lhx/coding-projects/decision-roadmap/ros/devel/lib/libglobal_planner.so
No symbol table info available.
.........
```
为了进一步定位问题，同时启用调试符号和 AddressSanitizer (ASAN)。这两者可以互补，帮助你更全面地分析和解决问题：

调试符号：提供详细的堆栈信息和变量值，便于在 GDB 中调试。

ASAN：检测内存错误（如越界访问、释放后使用、内存泄漏等），快速定位问题根源。

我在 `global_planner` 的 `CMakeLists.txt` 中同时启用调试符号和 ASAN：

```cmake
复制
# 在 catkin_package() 前添加以下内容

# 设置编译模式为 Debug
set(CMAKE_BUILD_TYPE Debug)

# 启用调试符号
add_compile_options(-g)

# 启用 AddressSanitizer
add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
add_link_options(-fsanitize=address)
```
然后删除build，devel，重新编译了工作空间，编译成功，但遇到如下警告：
```bash
......
/usr/bin/ld: /lib/x86_64-linux-gnu/libasan.so.5: warning: the use of `tmpnam_r' is dangerous, better use `mkstemp'
[100%] Built target dynamic_prm_node
[100%] Built target fg_drm_node
```
这似乎与debug模式无太大关联，继续debug。

在使用 AddressSanitizer (ASAN) 运行程序时，不需要通过 GDB 启动节点，因为 ASAN 会直接在程序崩溃时输出详细的错误信息（包括堆栈跟踪和内存错误类型）。因此，你只需正常启动节点即可。
在 launch 文件中，直接启动节点（无需 launch-prefix）：
```xml
<node pkg="autonomous_flight" type="fg_drm_node" name="fg_drm_node" output="screen" >
    <param name="robot_namespace" value="$(arg robot_namespace)"/>
</node>	
```
此外，ASAN 会显著增加内存占用。如果程序需要长时间运行，可调整 ASAN 选项：
```bash
# 在 launch 文件中添加环境变量
<env name="ASAN_OPTIONS" value="malloc_context_size=20:detect_leaks=0" />
```
OK, now launch the `run_fg_drm.launch`, and see what happens:
```bash
process[fg_drm_node-2]: started with pid [928118]
==928118==ASan runtime does not come first in initial library list; you should either link runtime to your application or manually preload it with LD_PRELOAD.
[fg_drm_node-2] process has died [pid 928118, exit code 1, cmd /home/lhx/coding-projects/decision-roadmap/ros/devel/lib/autonomous_flight/fg_drm_node /mavros/local_position/odom:=/CERLAB/quadcopter/odom /mavros/setpoint_raw/local:=/CERLAB/quadcopter/cmd_acc __name:=fg_drm_node __log:=/home/lhx/.ros/log/09337fbe-fa5d-11ef-a4c2-655dc81f0752/fg_drm_node-2.log].
```

需要在launch文件中显示添加 `LD_PRELOAD` 环境变量：

```xml
<!-- 添加环境变量预加载 ASan 库 -->
<env name="LD_PRELOAD" value="/lib/x86_64-linux-gnu/libasan.so.5" />
<env name="ASAN_OPTIONS" value="malloc_context_size=20:detect_leaks=0" />
```
`LD_PRELOAD` 的位置：
```bash
decision-roadmap/ros/devel/lib/autonomous_flight$ ldd fg_drm_node | grep libasan
	libasan.so.5 => /lib/x86_64-linux-gnu/libasan.so.5 (0x00007f7cfec96000)
```

用这个影响正常程序的运行，还是用gdb了
我同时添加了互斥锁和定时器的暂停逻辑，现在遇到段错误：
```cpp
Thread 1 "fg_drm_node" received signal SIGSEGV, Segmentation fault.
__gnu_cxx::new_allocator<std::shared_ptr<PRM::Node> >::allocate (this=<optimized out>, 
    __n=<optimized out>) at /usr/include/c++/9/ext/new_allocator.h:102
102	      allocate(size_type __n, const void* = static_cast<const void*>(0))
(gdb) bt full
#0  __gnu_cxx::new_allocator<std::shared_ptr<PRM::Node> >::allocate (this=<optimized out>, 
    __n=<optimized out>) at /usr/include/c++/9/ext/new_allocator.h:102
No locals.
#1  std::allocator_traits<std::allocator<std::shared_ptr<PRM::Node> > >::allocate (__a=..., 
    __n=<optimized out>) at /usr/include/c++/9/bits/alloc_traits.h:443
No locals.
#2  std::_Vector_base<std::shared_ptr<PRM::Node>, std::allocator<std::shared_ptr<PRM::Node> > >::_M_allocate (this=<optimized out>, __n=<optimized out>) at /usr/include/c++/9/bits/stl_vector.h:343
No locals.
#3  std::vector<std::shared_ptr<PRM::Node>, std::allocator<std::shared_ptr<PRM::Node> > >::_M_realloc_insert<std::shared_ptr<PRM::Node> const&> (this=this@entry=0x7fffff7ff0e0, 
    __position=non-dereferenceable iterator for std::vector)
    at /usr/include/c++/9/bits/vector.tcc:440
        __len = <optimized out>
        __old_start = <optimized out>
        __old_finish = <optimized out>
        __elems_before = <optimized out>
        __new_start = <optimized out>
        __new_finish = <optimized out>
#4  0x00007ffff737a742 in std::vector<std::shared_ptr<PRM::Node>, std::allocator<std::shared_ptr<PRM::Node> > >::push_back (__x=std::shared_ptr<PRM::Node> (empty) = {...}, this=0x7fffff7ff0e0)
    at /usr/include/c++/9/bits/stl_iterator.h:803
No locals.
#5  PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, n=
    std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, rootNode=
    std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {...}, 
    bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=56094)
    at /home/lhx/coding-projects/decision-roadmap/ros/src/global_planner/include/global_planner/PRMKDTree.cpp:121
        currDist = <optimized out>
        index = 0
        value = <optimized out>
        queryValue = <optimized out>
        ptr = std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {get() = 0x55555fb0c510}
        badSide = std::vector of length 0, capacity 0
#6  0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, 
    n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, 
    rootNode=std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {...}, 
    bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=56094)
    at /usr/include/c++/9/ext/atomicity.h:96
        index = 2
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 0
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 1, capacity 1 = {
          std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {get() = 0x55555fb0c510}}
#7  0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, 
    n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, 
    rootNode=std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {...}, 
    bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=56093)
    at /usr/include/c++/9/ext/atomicity.h:96
        index = 1
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 0
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 2, capacity 2 = {std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {
            get() = 0x55555fb0c510}}
#8  0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, 
    n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, 
    rootNode=std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {...}, 
    bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=56091)
    at /usr/include/c++/9/ext/atomicity.h:96
        index = 2
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 0
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 1, capacity 1 = {
          std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {get() = 0x55555fb0c510}}
......
#1915 0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, 
    n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, 
    rootNode=std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {...}, 
    bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=53231)
    at /usr/include/c++/9/ext/atomicity.h:96
        index = 1
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 0
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 2, capacity 2 = {std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {
            get() = 0x55555fb0c510}}
......
#37397 0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, rootNode=std::shared_ptr<PRM::Node> (use count 8, weak count 0) = {...}, bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=6) at /usr/include/c++/9/ext/atomicity.h:96
        index = 2
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 2
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 4, capacity 4 = {std::shared_ptr<PRM::Node> (use count 5, weak count 0) = {get() = 0x55555e363ee0}, std::shared_ptr<PRM::Node> (use count 6, weak count 0) = {get() = 0x55555e36c1d0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (use count 10, weak count 0) = {get() = 0x55555e3d7fb0}}
#37398 0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, rootNode=std::shared_ptr<PRM::Node> (use count 10, weak count 0) = {...}, bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=4) at /usr/include/c++/9/ext/atomicity.h:96
        index = 0
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 1
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 2, capacity 2 = {std::shared_ptr<PRM::Node> (use count 8, weak count 0) = {get() = 0x55555fa02650}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}}
#37399 0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, rootNode=std::shared_ptr<PRM::Node> (use count 14, weak count 0) = {...}, bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=3) at /usr/include/c++/9/ext/atomicity.h:96
        index = 2
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 2
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 4, capacity 4 = {std::shared_ptr<PRM::Node> (use count 12, weak count 0) = {get() = 0x55555faf4670}, std::shared_ptr<PRM::Node> (use count 10, weak count 0) = {get() = 0x55555fac6860}, std::shared_ptr<PRM::Node> (use count 10, weak count 0) = {get() = 0x55555f9831f0}, std::shared_ptr<PRM::Node> (use count 10, weak count 0) = {get() = 0x55555e3642e0}}
#37400 0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, rootNode=std::shared_ptr<PRM::Node> (empty) = {...}, bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=1) at /usr/include/c++/9/ext/atomicity.h:96
        index = 0
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 4
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 5, capacity 8 = {std::shared_ptr<PRM::Node> (use count 14, weak count 0) = {get() = 0x55555faf4c10}, std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555faf50d0}, std::shared_ptr<PRM::Node> (use count 9, weak count 0) = {get() = 0x55555faf6860}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (use count 7, weak count 0) = {get() = 0x55555fb45080}}
#37401 0x00007ffff737b29e in PRM::KDTree::kNearestNeighbor (this=this@entry=0x555555ae5e20, n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, num=num@entry=15) at /usr/include/c++/9/ext/atomicity.h:96
        nearestNeighborNode = std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555e3639e0}
        i = 2
        knn = std::vector of length 2, capacity 2 = {std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555f97c110}, std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555e3639e0}}
#37402 0x00007ffff73cad37 in globalPlanner::FGDRM::buildRoadMap (this=0x555555ae4f10) at /usr/include/c++/9/bits/shared_ptr.h:129
        fn = std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {get() = 0x55555e37b4b0}
        fnNeighbors = std::vector of length 2, capacity 2 = {std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555f97c110}, std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555e3639e0}}
        saturate = false
        regionSaturate = false
        countSample = <optimized out>
        n = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}
        newNodes = std::vector of length 1, capacity 1 = {std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {get() = 0x55555fb0c510}}
        sampleWeights = std::vector of length 4, capacity 4 = {36.511722346926256, 40.170912657140434, 36.506918826210459, 40.122199591642712}
        countFrontierFailure = 0
#37403 0x00007ffff73cbfc6 in globalPlanner::FGDRM::makePlan (this=0x555555ae4f10) at /home/lhx/coding-projects/decision-roadmap/ros/src/global_planner/include/global_planner/fg_drm.cpp:357
No locals.
#37404 0x00007ffff7d35cd0 in AutoFlight::fgDRMManager::getRoadmapServiceCB(global_planner::GetRoadmapRequest_<std::allocator<void> >&, global_planner::GetRoadmapResponse_<std::allocator<void> >&) () from /home/lhx/coding-projects/decision-roadmap/ros/devel/lib/libautonomous_flight.so
No symbol table info available.
#37405 0x00007ffff7d2c4bf in ros::ServiceCallbackHelperT<ros::ServiceSpec<global_planner::GetRoadmapRequest_<std::allocator<void> >, global_planner::GetRoadmapResponse_<std::allocator<void> > > >::call(ros::ServiceCallbackHelperCallParams&) () from /home/lhx/coding-projects/decision-roadmap/ros/devel/lib/libautonomous_flight.so
No symbol table info available.
#37406 0x00007ffff7ecc439 in ros::ServiceCallback::call() () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37407 0x00007ffff7f1d172 in ros::CallbackQueue::callOneCB(ros::CallbackQueue::TLS*) () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37408 0x00007ffff7f1e883 in ros::CallbackQueue::callAvailable(ros::WallDuration) () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37409 0x00007ffff7f71fcf in ros::SingleThreadedSpinner::spin(ros::CallbackQueue*) () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37410 0x00007ffff7f5a21f in ros::spin() () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37411 0x000055555555ec56 in main ()
No symbol table info available.
```

根本原因是 ​递归深度失控 和 ​智能指针循环引用，导致内存耗尽和分配失败。通过限制递归深度、优化内存使用、打破循环引用，可以彻底解决问题。
simplified error info:
```cpp
Thread 1 "fg_drm_node" received signal SIGSEGV, Segmentation fault.
__gnu_cxx::new_allocator<std::shared_ptr<PRM::Node> >::allocate (this=<optimized out>, 
    __n=<optimized out>) at /usr/include/c++/9/ext/new_allocator.h:102
102	      allocate(size_type __n, const void* = static_cast<const void*>(0))
(gdb) bt full
#0  __gnu_cxx::new_allocator<std::shared_ptr<PRM::Node> >::allocate (this=<optimized out>, 
    __n=<optimized out>) at /usr/include/c++/9/ext/new_allocator.h:102
No locals.
#1  std::allocator_traits<std::allocator<std::shared_ptr<PRM::Node> > >::allocate (__a=..., 
    __n=<optimized out>) at /usr/include/c++/9/bits/alloc_traits.h:443
No locals.
#2  std::_Vector_base<std::shared_ptr<PRM::Node>, std::allocator<std::shared_ptr<PRM::Node> > >::_M_allocate (this=<optimized out>, __n=<optimized out>) at /usr/include/c++/9/bits/stl_vector.h:343
No locals.
#3  std::vector<std::shared_ptr<PRM::Node>, std::allocator<std::shared_ptr<PRM::Node> > >::_M_realloc_insert<std::shared_ptr<PRM::Node> const&> (this=this@entry=0x7fffff7ff0e0, 
    __position=non-dereferenceable iterator for std::vector)
    at /usr/include/c++/9/bits/vector.tcc:440
        __len = <optimized out>
        __old_start = <optimized out>
        __old_finish = <optimized out>
        __elems_before = <optimized out>
        __new_start = <optimized out>
        __new_finish = <optimized out>
#4  0x00007ffff737a742 in std::vector<std::shared_ptr<PRM::Node>, std::allocator<std::shared_ptr<PRM::Node> > >::push_back (__x=std::shared_ptr<PRM::Node> (empty) = {...}, this=0x7fffff7ff0e0)
    at /usr/include/c++/9/bits/stl_iterator.h:803
No locals.
#5  PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, n=
    std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, rootNode=
    std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {...}, 
    bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=56094)
    at /home/lhx/coding-projects/decision-roadmap/ros/src/global_planner/include/global_planner/PRMKDTree.cpp:121
        currDist = <optimized out>
        index = 0
        value = <optimized out>
        queryValue = <optimized out>
        ptr = std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {get() = 0x55555fb0c510}
        badSide = std::vector of length 0, capacity 0
#6  0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, 
    n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, 
    rootNode=std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {...}, 
    bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=56094)
    at /usr/include/c++/9/ext/atomicity.h:96
        index = 2
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 0
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 1, capacity 1 = {
          std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {get() = 0x55555fb0c510}}
......
#37399 0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, rootNode=std::shared_ptr<PRM::Node> (use count 14, weak count 0) = {...}, bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=3) at /usr/include/c++/9/ext/atomicity.h:96
        index = 2
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 2
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 4, capacity 4 = {std::shared_ptr<PRM::Node> (use count 12, weak count 0) = {get() = 0x55555faf4670}, std::shared_ptr<PRM::Node> (use count 10, weak count 0) = {get() = 0x55555fac6860}, std::shared_ptr<PRM::Node> (use count 10, weak count 0) = {get() = 0x55555f9831f0}, std::shared_ptr<PRM::Node> (use count 10, weak count 0) = {get() = 0x55555e3642e0}}
#37400 0x00007ffff737a435 in PRM::KDTree::nearestNeighbor (this=0x555555ae5e20, n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, rootNode=std::shared_ptr<PRM::Node> (empty) = {...}, bestNode=std::shared_ptr<PRM::Node> (use count 37403, weak count 0) = {...}, depth=1) at /usr/include/c++/9/ext/atomicity.h:96
        index = 0
        value = <optimized out>
        queryValue = <optimized out>
        bestPossibleDist = <optimized out>
        searchIdx = <optimized out>
        ptr = <optimized out>
        i = 4
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 5, capacity 8 = {std::shared_ptr<PRM::Node> (use count 14, weak count 0) = {get() = 0x55555faf4c10}, std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555faf50d0}, std::shared_ptr<PRM::Node> (use count 9, weak count 0) = {get() = 0x55555faf6860}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (use count 7, weak count 0) = {get() = 0x55555fb45080}}
#37401 0x00007ffff737b29e in PRM::KDTree::kNearestNeighbor (this=this@entry=0x555555ae5e20, n=std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {...}, num=num@entry=15) at /usr/include/c++/9/ext/atomicity.h:96
        nearestNeighborNode = std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555e3639e0}
        i = 2
        knn = std::vector of length 2, capacity 2 = {std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555f97c110}, std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555e3639e0}}
#37402 0x00007ffff73cad37 in globalPlanner::FGDRM::buildRoadMap (this=0x555555ae4f10) at /usr/include/c++/9/bits/shared_ptr.h:129
        fn = std::shared_ptr<PRM::Node> (use count 37398, weak count 0) = {get() = 0x55555e37b4b0}
        fnNeighbors = std::vector of length 2, capacity 2 = {std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555f97c110}, std::shared_ptr<PRM::Node> (use count 11, weak count 0) = {get() = 0x55555e3639e0}}
        saturate = false
        regionSaturate = false
        countSample = <optimized out>
        n = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}
        newNodes = std::vector of length 1, capacity 1 = {std::shared_ptr<PRM::Node> (use count 112175, weak count 0) = {get() = 0x55555fb0c510}}
        sampleWeights = std::vector of length 4, capacity 4 = {36.511722346926256, 40.170912657140434, 36.506918826210459, 40.122199591642712}
        countFrontierFailure = 0
#37403 0x00007ffff73cbfc6 in globalPlanner::FGDRM::makePlan (this=0x555555ae4f10) at /home/lhx/coding-projects/decision-roadmap/ros/src/global_planner/include/global_planner/fg_drm.cpp:357
No locals.
#37404 0x00007ffff7d35cd0 in AutoFlight::fgDRMManager::getRoadmapServiceCB(global_planner::GetRoadmapRequest_<std::allocator<void> >&, global_planner::GetRoadmapResponse_<std::allocator<void> >&) () from /home/lhx/coding-projects/decision-roadmap/ros/devel/lib/libautonomous_flight.so
No symbol table info available.
#37405 0x00007ffff7d2c4bf in ros::ServiceCallbackHelperT<ros::ServiceSpec<global_planner::GetRoadmapRequest_<std::allocator<void> >, global_planner::GetRoadmapResponse_<std::allocator<void> > > >::call(ros::ServiceCallbackHelperCallParams&) () from /home/lhx/coding-projects/decision-roadmap/ros/devel/lib/libautonomous_flight.so
No symbol table info available.
#37406 0x00007ffff7ecc439 in ros::ServiceCallback::call() () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37407 0x00007ffff7f1d172 in ros::CallbackQueue::callOneCB(ros::CallbackQueue::TLS*) () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37408 0x00007ffff7f1e883 in ros::CallbackQueue::callAvailable(ros::WallDuration) () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37409 0x00007ffff7f71fcf in ros::SingleThreadedSpinner::spin(ros::CallbackQueue*) () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37410 0x00007ffff7f5a21f in ros::spin() () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#37411 0x000055555555ec56 in main ()
No symbol table info available.
```
Original version of nearestNeighbor(), kNearestNeighbor()
```cpp
    std::shared_ptr<Node> KDTree::nearestNeighbor(std::shared_ptr<Node> n,
	                             				  std::shared_ptr<Node> rootNode, 
	                             				  std::shared_ptr<Node> bestNode,
	                             				  int depth){
		std::shared_ptr<Node> ptr;
		if (rootNode == NULL){
			ptr = this->root_;
		}
		else{
			ptr = rootNode;
		}

		// Search good side and store bad side
		std::vector<std::shared_ptr<Node>> badSide;
		while (ptr != NULL){
			double currDist = (n->pos - ptr->pos).norm(); 

			// avoid finding the same node
	 		if (ptr == n){
				currDist = std::numeric_limits<double>::infinity();
			}
			// avoid finding node in this list
			// for (std::shared_ptr<Node> nt: this->notTarget_){
			// 	if (ptr == nt){
			// 		currDist = std::numeric_limits<double>::infinity();
			// 		break;
			// 	}
			// }

			// avoid target here
			if (this->notTargetTemp_.find(ptr) != this->notTargetTemp_.end()){
				currDist = std::numeric_limits<double>::infinity();
			}

			if (this->notTargetPerm_.find(ptr) != this->notTargetPerm_.end()){
				currDist = std::numeric_limits<double>::infinity();
			}

			if (currDist < this->leastDistNN_){
				bestNode = ptr;
				this->leastDistNN_ = currDist;
			}

			int index = int(depth % 3);
			double value = ptr->pos(index);
			double queryValue = n->pos(index);

			// if less than, bad side is the right side. Otherwise left side.
			if (queryValue < value){
				badSide.push_back(ptr->right);
				ptr = ptr->left;
			}
			else{
				badSide.push_back(ptr->left);
				ptr = ptr->right;
			}
			++depth;
		}


		// Search the previous bad side (from the latest to oldest)
		for (size_t i=0; i<badSide.size(); ++i){
			int searchIdx = int(badSide.size()) - i - 1;
			std::shared_ptr<Node> ptr = badSide[searchIdx];
			if (ptr == NULL){
				--depth;
				continue;
			}
			else{
				// recursively search the bad side's parent node
				int index = int((depth-1) % 3);
				double value = ptr->treeParent->pos(index);
				double queryValue = n->pos(index);

				double bestPossibleDist = std::abs(value - queryValue);
				if (bestPossibleDist >= this->leastDistNN_){
					--depth;
					continue;
				}
				else{
					bestNode = this->nearestNeighbor(n, ptr, bestNode, depth);
					--depth;
				}
			}
		}

		if (rootNode == NULL){
			this->leastDistNN_ = std::numeric_limits<double>::infinity(); 
		}
		return bestNode;
	}
	
	// Returns the k-nearest neighbor in ascending order
	std::vector<std::shared_ptr<Node>> KDTree::kNearestNeighbor(std::shared_ptr<Node> n, int num){
		std::vector<std::shared_ptr<Node>> knn;
		num = std::min(this->size_-1, num);
		for (int i=0; i<num; ++i){
			std::shared_ptr<Node> nearestNeighborNode = nearestNeighbor(n);
			if (nearestNeighborNode == NULL){
				if (n != NULL){
					cout << "invalid node: " << n->pos << endl;
				}
				else{
					cout << "NULL node." << endl;
				}
				cout << "find null pointer at " << i << endl;
			}
			knn.push_back(nearestNeighborNode);
			// this->notTarget_.push_back(nearestNeighborNode);
			this->notTargetTemp_.insert(nearestNeighborNode);
		}
		// this->notTarget_.clear();
		this->notTargetTemp_.clear();
		return knn;
	}
```

Modified version by DeepSeek
```cpp
    std::shared_ptr<Node> KDTree::nearestNeighbor(std::shared_ptr<Node> n,
                                             std::shared_ptr<Node> rootNode, 
                                             std::shared_ptr<Node> bestNode,
                                             int depth) {
		// 初始化当前节点指针
		std::shared_ptr<Node> ptr = rootNode ? rootNode : this->root_;
		if (!ptr) return bestNode;  // 防御空树

		// 使用pair存储节点及其对应的深度
		std::vector<std::pair<std::shared_ptr<Node>, int>> badSide;
		badSide.reserve(16);  // 预分配空间优化

		// 主搜索路径
		while (ptr) {
			// 递归深度保护（可根据实际树高调整阈值）
			if (depth > 1000) {
				throw std::runtime_error("Excessive recursion depth: " + std::to_string(depth));
			}

			// 距离计算逻辑
			double currDist = (n->pos - ptr->pos).norm();
			
			// 排除当前节点自身的检查
			if (ptr == n || 
				this->notTargetTemp_.count(ptr) || 
				this->notTargetPerm_.count(ptr)) 
			{
				currDist = std::numeric_limits<double>::infinity();
			}

			// 更新最近节点
			if (currDist < this->leastDistNN_) {
				bestNode = ptr;
				this->leastDistNN_ = currDist;
			}

			// 计算分割维度
			const int current_depth = depth;  // 冻结当前深度
			const int index = current_depth % 3;
			const double value = ptr->pos[index];
			const double queryValue = n->pos[index];

			// 记录badSide时保存正确的子节点深度
			if (queryValue < value) {
				badSide.emplace_back(ptr->right, current_depth + 1);
				ptr = ptr->left;
			} else {
				badSide.emplace_back(ptr->left, current_depth + 1);
				ptr = ptr->right;
			}
			
			++depth;  // 仅递增当前路径的深度
		}

		// 处理badSide（反向迭代）
		for (auto it = badSide.rbegin(); it != badSide.rend(); ++it) {
			auto& [node, node_depth] = *it;
			if (!node) continue;

			// 获取父节点分割平面
			if (!node->treeParent) {  // 防御空指针
				bestNode = nearestNeighbor(n, node, bestNode, node_depth);
				continue;
			}

			const int parent_index = (node_depth - 1) % 3;
			const double parent_value = node->treeParent->pos[parent_index];
			const double query_value = n->pos[parent_index];

			// 检查是否需要搜索该子树
			if (std::abs(parent_value - query_value) < this->leastDistNN_) {
				bestNode = nearestNeighbor(n, node, bestNode, node_depth);
			}
		}

		// 重置全局最近距离（仅当搜索从根节点开始时）
		if (!rootNode) {
			this->leastDistNN_ = std::numeric_limits<double>::infinity();
		}
		
		return bestNode;
	}
	
	// Returns the k-nearest neighbor in ascending order
	std::vector<std::shared_ptr<Node>> KDTree::kNearestNeighbor(std::shared_ptr<Node> n, int num) {
		if (!n) {
			throw std::invalid_argument("Input node cannot be null");
		}
		
		std::vector<std::shared_ptr<Node>> knn;
		num = std::min(this->size_-1, num);
		
		try {
			for (int i=0; i<num; ++i){
				auto nearest = nearestNeighbor(n);
				if (!nearest) {
					throw std::runtime_error("Failed to find neighbor at iteration " + std::to_string(i));
				}
				knn.push_back(nearest);
				this->notTargetTemp_.insert(nearest);
			}
		} catch (const std::exception& e) {
			this->notTargetTemp_.clear();
			throw;  // 重新抛出异常
		}
		
		this->notTargetTemp_.clear();
		return knn;
	}
```

再次遇到报错：
```cpp
[ERROR] [1741261072.150031273, 3818.792000000]: [fgDRMManager] Exception in getRoadmapServiceCB: Excessive recursion depth: 1001
terminate called after throwing an instance of 'std::runtime_error'
  what():  Excessive recursion depth: 1001

Thread 1 "fg_drm_node" received signal SIGABRT, Aborted.
__GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
50	../sysdeps/unix/sysv/linux/raise.c: No such file or directory.
(gdb) bt full
#0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
        set = {__val = {4096, 73014444033, 140737488231520, 3, 0, 0, 7959393510860551283, 
            8246765328066767220, 140737471607407, 140737488344624, 140737488344632, 
            140737488344640, 140737488344648, 0, 140737488344664, 0}}
        pid = <optimized out>
        tid = <optimized out>
        ret = <optimized out>
#1  0x00007ffff791d859 in __GI_abort () at abort.c:79
        save_stage = 1
        act = {__sigaction_handler = {sa_handler = 0x7ffff7ae85c0 <_IO_2_1_stderr_>, 
            sa_sigaction = 0x7ffff7ae85c0 <_IO_2_1_stderr_>}, sa_mask = {__val = {140737348797888, 
              1, 140737348798019, 3432, 140737347369297, 31, 10, 140737348797888, 140737145382464, 
              140737349723008, 140737488233568, 140737488233520, 140737347370643, 10, 
              140737348797888, 140737145382464}}, sa_flags = -138725980, 
          sa_restorer = 0x7ffff7ae8780 <stderr>}
        sigs = {__val = {32, 0 <repeats 15 times>}}
#2  0x00007ffff7ba88d1 in ?? () from /lib/x86_64-linux-gnu/libstdc++.so.6
No symbol table info available.
#3  0x00007ffff7bb437c in ?? () from /lib/x86_64-linux-gnu/libstdc++.so.6
No symbol table info available.
#4  0x00007ffff7bb43e7 in std::terminate() () from /lib/x86_64-linux-gnu/libstdc++.so.6
No symbol table info available.
#5  0x00007ffff7bb4699 in __cxa_throw () from /lib/x86_64-linux-gnu/libstdc++.so.6
No symbol table info available.
#6  0x00007ffff736f517 in PRM::KDTree::nearestNeighbor (this=<optimized out>, n=..., rootNode=
    std::shared_ptr<struct PRM::Node> (use count 676, weak count 0) = {...}, 
    bestNode=std::shared_ptr<struct PRM::Node> (empty) = {...}, depth=<optimized out>)
    at /usr/include/c++/9/ext/new_allocator.h:89
        currDist = <optimized out>
        current_depth = <optimized out>
        value = <optimized out>
        index = <optimized out>
        queryValue = <optimized out>
        ptr = std::shared_ptr<PRM::Node> (use count 676, weak count 0) = {get() = <optimized out>}
        badSide = std::vector of length 2, capacity 16 = {{
            first = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, second = 1000}, {
            first = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, second = 1001}}
#7  0x00007ffff737a92b in PRM::KDTree::nearestNeighbor (this=<optimized out>, n=..., 
    rootNode=std::shared_ptr<struct PRM::Node> (use count 676, weak count 0) = {...}, 
    bestNode=std::shared_ptr<struct PRM::Node> (empty) = {...}, depth=<optimized out>)
    at /usr/include/c++/9/ext/atomicity.h:96
        node = std::shared_ptr<PRM::Node> (use count 676, weak count 0) = {get() = 0x55555c4e2da0}
        node_depth = @0x55555ca5f860: 999
        parent_value = <optimized out>
        parent_index = <optimized out>
        query_value = <optimized out>
        it = {<std::iterator<std::random_access_iterator_tag, std::pair<std::shared_ptr<PRM::Node>, int>, long, std::pair<std::shared_ptr<PRM::Node>, int>*, std::pair<std::shared_ptr<PRM::Node>, int>&>> = {<No data fields>}, current = 
  {first = <error reading variable: Cannot access memory at address 0x1008>, second = 512}}
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 3, capacity 16 = {{
            first = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, second = 997}, {
            first = std::shared_ptr<PRM::Node> (empty) = {get
            () = 0x0}, second = 998}, {
......
#338 0x00007ffff737a92b in PRM::KDTree::nearestNeighbor (this=<optimized out>, n=..., rootNode=std::shared_ptr<struct PRM::Node> (use count 676, weak count 0) = {...}, bestNode=std::shared_ptr<struct PRM::Node> (empty) = {...}, depth=<optimized out>) at /usr/include/c++/9/ext/atomicity.h:96
        node = std::shared_ptr<PRM::Node> (use count 676, weak count 0) = {get() = 0x55555c4e2da0}
        node_depth = @0x55555ca5e280: 6
        parent_value = <optimized out>
        parent_index = <optimized out>
        query_value = <optimized out>
        it = {<std::iterator<std::random_access_iterator_tag, std::pair<std::shared_ptr<PRM::Node>, int>, long, std::pair<std::shared_ptr<PRM::Node>, int>*, std::pair<std::shared_ptr<PRM::Node>, int>&>> = {<No data fields>}, current = {first = <error reading variable: Cannot access memory at address 0xc0045cdfd3bbb452>, second = 472925990}}
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 3, capacity 16 = {{first = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, second = 4}, {first = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, second = 5}, {first = std::shared_ptr<PRM::Node> (use count 676, weak count 0) = {get() = 0x55555c4e2da0}, second = 6}}
#339 0x00007ffff737a92b in PRM::KDTree::nearestNeighbor (this=this@entry=0x555555ae5e50, n=std::shared_ptr<struct PRM::Node> (use count 335, weak count 0) = {...}, rootNode=std::shared_ptr<struct PRM::Node> (empty) = {...}, bestNode=std::shared_ptr<struct PRM::Node> (empty) = {...}, depth=<optimized out>, depth@entry=0) at /usr/include/c++/9/ext/atomicity.h:96
        node = std::shared_ptr<PRM::Node> (use count 676, weak count 0) = {get() = 0x55555c4e2da0}
        node_depth = @0x55555ca5e410: 3
        parent_value = <optimized out>
        parent_index = <optimized out>
        query_value = <optimized out>
        it = {<std::iterator<std::random_access_iterator_tag, std::pair<std::shared_ptr<PRM::Node>, int>, long, std::pair<std::shared_ptr<PRM::Node>, int>*, std::pair<std::shared_ptr<PRM::Node>, int>&>> = {<No data fields>}, current = {first = <error reading variable: Cannot access memory at address 0xc00ca8f62d4764ba>, second = -457670371}}
        ptr = std::shared_ptr<PRM::Node> (empty) = {get() = <optimized out>}
        badSide = std::vector of length 3, capacity 16 = {{first = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, second = 1}, {first = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, second = 2}, {first = std::shared_ptr<PRM::Node> (use count 676, weak count 0) = {get() = 0x55555c4e2da0}, second = 3}}
#340 0x00007ffff73c9e91 in globalPlanner::FGDRM::waypointUpdateCB (this=0x555555ae4f10) at /usr/include/c++/9/bits/shared_ptr.h:129
        temp_goal = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}
        lastNavWaypoint = {x = -9.31257057, y = -4.55021048, z = 0.944233716, intensity = 0}
        currPos = std::shared_ptr<PRM::Node> (use count 335, weak count 0) = {get() = 0x55555c47d650}
        start = std::shared_ptr<PRM::Node> (use count 4, weak count 0) = {get() = 0x55555ca6c970}
        path = std::vector of length 0, capacity 5864033411320
        waypoint = {header = {seq = 1, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 4154294453, static MIN = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 1, static MIN = <same as static member of an already seen type>, static MAX = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 4294967295, nsec = 999999999, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static MAX = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 4294967295, nsec = 999999999, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, frame_id = ""}, point = {x = 3.6263810704003112e-317, y = 1.0144126394099982e-315, z = 6.9533326056490441e-310}}
        lock = {_M_device = @0x555555ae55e8}
        __PRETTY_FUNCTION__ = "void globalPlanner::FGDRM::waypointUpdateCB(const ros::TimerEvent&)"
#341 0x00007ffff7efb608 in ros::TimerManager<ros::Time, ros::Duration, ros::TimerEvent>::TimerQueueCallback::call() () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#342 0x00007ffff7f1d172 in ros::CallbackQueue::callOneCB(ros::CallbackQueue::TLS*) () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#343 0x00007ffff7f1e883 in ros::CallbackQueue::callAvailable(ros::WallDuration) () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#344 0x00007ffff7f71fcf in ros::SingleThreadedSpinner::spin(ros::CallbackQueue*) () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#345 0x00007ffff7f5a21f in ros::spin() () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#346 0x000055555555ec56 in main ()
No symbol table info available.
```

再次修改后的KDTree代码：
修改 `insert` 函数:
```cpp
void KDTree::insert(std::shared_ptr<Node> n) {
    n->left = n->right = nullptr;
    n->treeParent.reset();

    std::function<std::shared_ptr<Node>(std::shared_ptr<Node>, int)> insertRecursive;
    insertRecursive = [&](std::shared_ptr<Node> parent, int depth) -> std::shared_ptr<Node> {
        if (!parent) {
            ++size_;
            return n;
        }

        int axis = depth % 3;
        if (n->pos[axis] < parent->pos[axis]) {
            parent->left = insertRecursive(parent->left, depth + 1);
            parent->left->treeParent = parent;
        } else {
            parent->right = insertRecursive(parent->right, depth + 1);
            parent->right->treeParent = parent;
        }
        return parent;
    };

    root_ = insertRecursive(root_, 0);
}
```
Original `insert` 函数：
```cpp
    void KDTree::insert(std::shared_ptr<Node> n){
		// set the newly inserted node child to NULL
		n->left = NULL;
		n->right = NULL;

		// if tree is empty, we add the node as root node
		if (this->size_ == 0){
			this->root_ = n;
			++this->size_;
			return;
		}
		else{
			std::shared_ptr<Node> ptr = this->root_;
			int depth = 0;
			while (true){
				int index = int(depth % 3);
				double value = ptr->pos(index);
				double insertValue = n->pos(index);

				if (insertValue >= value){
					if (ptr->right == NULL){
						ptr->right = n;
						n->treeParent = ptr;
						++this->size_;
						return;
					}
					ptr = ptr->right;
				}
				else{
					if (ptr->left == NULL){
						ptr->left = n;
						n->treeParent = ptr;
						++this->size_;
						return;
					}
					ptr = ptr->left;
				}
				++depth;
			}
		}
	}
```
Newest: use the insert() modfied from GPT and add kdtree log:
```cpp
std::weak_ptr<Node> treeParent;

void KDTree::insert(std::shared_ptr<Node> n) {
    // set child pointers to NULL (optional if Node constructor does this)
    n->left = nullptr;
    n->right = nullptr;

    // if tree is empty, add node as root
    if (this->size_ == 0) {
        this->root_ = n;
        this->size_++;
        return;
    }

    std::shared_ptr<Node> ptr = this->root_;
    int depth = 0;

    while (true) {
        int index = depth % 3; // Use dimension_ if supporting more than 3D
        double value = ptr->pos(index);
        double insertValue = n->pos(index);

        if (insertValue > value) {
            if (ptr->right == nullptr) {
                ptr->right = n;
                n->treeParent = ptr;
                this->size_++;
                return;
            }
            ptr = ptr->right;
        } else if (insertValue < value) {
            if (ptr->left == nullptr) {
                ptr->left = n;
                n->treeParent = ptr;
                this->size_++;
                return;
            }
            ptr = ptr->left;
        } else { // Avoid infinite loop
            return;
        }

        depth++;
    }
}
```
New error
```cpp
[KD-Tree]: Size = 124 Height = 11
[KD-Tree]: Size = 128 Height = 11
[ERROR] [1741332541.580030221, 1893.159000000]: [fgDRMManager] Exception in getRoadmapServiceCB: Failed to find neighbor at iteration 0

Thread 1 "fg_drm_node" received signal SIGSEGV, Segmentation fault.
PRM::AStar (roadmap=std::shared_ptr<PRM::KDTree> (use count 1, weak count 0) = {...}, 
    start=std::shared_ptr<PRM::Node> (empty) = {...}, goal=std::shared_ptr<PRM::Node> (empty) = {...}, 
    map=warning: RTTI symbol not found for class 'std::_Sp_counted_ptr<mapManager::occMap*, (__gnu_cxx::_Lock_policy)2>'
warning: RTTI symbol not found for class 'std::_Sp_counted_ptr<mapManager::occMap*, (__gnu_cxx::_Lock_policy)2>'
std::shared_ptr<mapManager::occMap> (use count 2, weak count 0) = {...})
    at /usr/include/c++/9/bits/stl_vector.h:94
94		_Vector_impl_data() _GLIBCXX_NOEXCEPT
(gdb) bt full
#0  PRM::AStar (roadmap=std::shared_ptr<PRM::KDTree> (use count 1, weak count 0) = {...}, 
    start=std::shared_ptr<PRM::Node> (empty) = {...}, goal=std::shared_ptr<PRM::Node> (empty) = {...}, 
    map=warning: RTTI symbol not found for class 'std::_Sp_counted_ptr<mapManager::occMap*, (__gnu_cxx::_Lock_policy)2>'
warning: RTTI symbol not found for class 'std::_Sp_counted_ptr<mapManager::occMap*, (__gnu_cxx::_Lock_policy)2>'
std::shared_ptr<mapManager::occMap> (use count 2, weak count 0) = {...})
    at /usr/include/c++/9/bits/stl_vector.h:94
        path = std::vector of length 0, capacity 0
        open = std::priority_queue wrapping: std::vector of length 0, capacity -8796093021253
        close = std::unordered_set with 140737488340112 elements = {
          [0] = std::shared_ptr<PRM::Node> (use count 49, weak count -1) = {
            get() = 0x31}<error reading variable: Cannot access memory at address 0x3fec7d8712760074>...}
        record = std::vector of length 8796093021252, capacity 8796093021252 = {
          <error reading variable record (Cannot access memory at address 0x0)>
        findPath = <optimized out>
        ptr = std::shared_ptr<PRM::Node> (use count -15312, weak count 32766) = {get() = 0x7fffffffc430}
#1  0x00007ffff73c8d99 in globalPlanner::FGDRM::waypointUpdateCB (this=0x555555ae4f10)
    at /home/lhx/coding-projects/decision-roadmap/ros/src/global_planner/include/global_planner/fg_drm.cpp:1015
        temp_goal = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}
        lastNavWaypoint = {x = -15.4478359, y = -0.848773062, z = 0.977405488, intensity = 0}
        currPos = std::shared_ptr<PRM::Node> (use count 1, weak count 0) = {get() = 0x555560ec2e40}
        start = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}
        path = std::vector of length 0, capacity 0
        waypoint = {header = {seq = 1, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, 
                nsec = 4154294453, static MIN = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, 
                    nsec = 1, static MIN = <same as static member of an already seen type>, 
                    static MAX = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 4294967295, 
                        nsec = 999999999, static MIN = <same as static member of an already seen type>, 
                        static MAX = <same as static member of an already seen type>, 
                        static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, 
                            static MIN = <same as static member of an already seen type>, 
                            static MAX = <same as static member of an already seen type>, 
                            static ZERO = <same as static member of an already seen type>, 
                            static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, 
                                nsec = 0, static MIN = <same as static member of an already seen type>, 
                                static MAX = <same as static member of an already seen type>, 
                                static ZERO = <same as static member of an already seen type>, 
                                static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {
                            sec = 0, nsec = 0, 
                            static MIN = <same as static member of an already seen type>, 
                            static MAX = <same as static member of an already seen type>, 
                            static ZERO = <same as static member of an already seen type>, 
                            static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, 
                        nsec = 0, static MIN = <same as static member of an already seen type>, 
                        static MAX = <same as static member of an already seen type>, 
                        static ZERO = <same as static member of an already seen type>, 
                        static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, 
                            nsec = 0, static MIN = <same as static member of an already seen type>, 
                            static MAX = <same as static member of an already seen type>, 
                            static ZERO = <same as static member of an already seen type>, 
                            static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {
                        sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, 
                        static MAX = <same as static member of an already seen type>, 
                        static ZERO = <same as static member of an already seen type>, 
                        static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static MAX = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 4294967295, 
                    nsec = 999999999, static MIN = <same as static member of an already seen type>, 
                    static MAX = <same as static member of an already seen type>, 
                    static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, 
                        static MIN = <same as static member of an already seen type>, 
                        static MAX = <same as static member of an already seen type>, 
                        static ZERO = <same as static member of an already seen type>, 
                        static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, 
                            nsec = 0, static MIN = <same as static member of an already seen type>, 
                            static MAX = <same as static member of an already seen type>, 
                            static ZERO = <same as static member of an already seen type>, 
                            static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {
                        sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, 
                        static MAX = <same as static member of an already seen type>, 
                        static ZERO = <same as static member of an already seen type>, 
                        static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, 
                    nsec = 0, static MIN = <same as static member of an already seen type>, 
                    static MAX = <same as static member of an already seen type>, 
                    static ZERO = <same as static member of an already seen type>, 
                    static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, 
                        nsec = 0, static MIN = <same as static member of an already seen type>, 
                        static MAX = <same as static member of an already seen type>, 
                        static ZERO = <same as static member of an already seen type>, 
                        static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {
                    sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, 
                    static MAX = <same as static member of an already seen type>, 
                    static ZERO = <same as static member of an already seen type>, 
                    static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, frame_id = ""}, point = {x = 3.6616914480429392e-317, 
            y = 4.011033996579903e-315, z = 6.9533326040609195e-310}}
        lock = {_M_device = @0x555555ae55e8}
        __PRETTY_FUNCTION__ = "void globalPlanner::FGDRM::waypointUpdateCB(const ros::TimerEvent&)"
#2  0x00007ffff7efb608 in ros::TimerManager<ros::Time, ros::Duration, ros::TimerEvent>::TimerQueueCallback::call() () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#3  0x00007ffff7f1d172 in ros::CallbackQueue::callOneCB(ros::CallbackQueue::TLS*) ()
   from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#4  0x00007ffff7f1e883 in ros::CallbackQueue::callAvailable(ros::WallDuration) ()
   from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#5  0x00007ffff7f71fcf in ros::SingleThreadedSpinner::spin(ros::CallbackQueue*) ()
   from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#6  0x00007ffff7f5a21f in ros::spin() () from /opt/ros/noetic/lib/libroscpp.so
No symbol table info available.
#7  0x000055555555ec56 in main ()
No symbol table info available.
```
save the nearestNeighbor() and kNearestNeighbor() with this error:
```cpp
std::shared_ptr<Node> KDTree::nearestNeighbor(std::shared_ptr<Node> n,
                                             std::shared_ptr<Node> rootNode, 
                                             std::shared_ptr<Node> bestNode,
                                             int depth) {
		// 初始化当前节点指针
		std::shared_ptr<Node> ptr = rootNode ? rootNode : this->root_;
		if (!ptr) return bestNode;  // 防御空树

		// 使用pair存储节点及其对应的深度
		std::vector<std::pair<std::shared_ptr<Node>, int>> badSide;
		badSide.reserve(16);  // 预分配空间优化

		// 主搜索路径
		while (ptr) {
			// 递归深度保护（可根据实际树高调整阈值）
			if (depth > 100) {
				// print tree structure
				printTree(this->root_, 0);
				std::cout << "\033[1;31m[KD-Tree]:" << " Size = " << this->getSize() << " Height = " << this->checkBalance() << "\033[0m" << std::endl;
				throw std::runtime_error("Excessive recursion depth: " + std::to_string(depth));
			}

			// 距离计算逻辑
			double currDist = (n->pos - ptr->pos).norm();
			
			// 排除当前节点自身的检查
			if (ptr == n || 
				this->notTargetTemp_.count(ptr) || 
				this->notTargetPerm_.count(ptr)) 
			{
				currDist = std::numeric_limits<double>::infinity();
			}

			// 更新最近节点
			if (currDist < this->leastDistNN_) {
				bestNode = ptr;
				this->leastDistNN_ = currDist;
			}

			// 计算分割维度
			const int current_depth = depth;  // 冻结当前深度
			const int index = current_depth % 3;
			const double value = ptr->pos[index];
			const double queryValue = n->pos[index];

			// 记录badSide时保存正确的子节点深度
			if (queryValue < value) {
				badSide.emplace_back(ptr->right, current_depth + 1);
				ptr = ptr->left;
			} else {
				badSide.emplace_back(ptr->left, current_depth + 1);
				ptr = ptr->right;
			}
			
			++depth;  // 仅递增当前路径的深度
		}

		// 处理badSide（反向迭代）
		for (auto it = badSide.rbegin(); it != badSide.rend(); ++it) {
			auto& [node, node_depth] = *it;
			if (!node) continue;

			// 获取父节点分割平面
			if (!node->treeParent) {  // 防御空指针
				bestNode = nearestNeighbor(n, node, bestNode, node_depth);
				continue;
			}

			const int parent_index = (node_depth - 1) % 3;
			const double parent_value = node->treeParent->pos[parent_index];
			const double query_value = n->pos[parent_index];

			// 检查是否需要搜索该子树
			if (std::abs(parent_value - query_value) < this->leastDistNN_) {
				bestNode = nearestNeighbor(n, node, bestNode, node_depth);
			}
		}

		// 重置全局最近距离（仅当搜索从根节点开始时）
		if (!rootNode) {
			this->leastDistNN_ = std::numeric_limits<double>::infinity();
		}
		
		return bestNode;
	}
	
	// Returns the k-nearest neighbor in ascending order
	std::vector<std::shared_ptr<Node>> KDTree::kNearestNeighbor(std::shared_ptr<Node> n, int num) {
		if (!n) {
			throw std::invalid_argument("Input node cannot be null");
		}
		
		std::vector<std::shared_ptr<Node>> knn;
		num = std::min(this->size_-1, num);
		
		try {
			for (int i=0; i<num; ++i){
				auto nearest = nearestNeighbor(n);
				if (!nearest) {
					throw std::runtime_error("Failed to find neighbor at iteration " + std::to_string(i));
				}
				knn.push_back(nearest);
				this->notTargetTemp_.insert(nearest);
			}
		} catch (const std::exception& e) {
			this->notTargetTemp_.clear();
			throw;  // 重新抛出异常
		}
		
		this->notTargetTemp_.clear();
		return knn;
	}
```

Now this version of modification: Use the nn() and knn() proposed by GPT.
See what happens:

This time the training successfully done!
nearestNeighbor() and kNearestNeighbor() modified by GPT:
```cpp
// 辅助函数：递归实现最近邻搜索
std::shared_ptr<Node> KDTree::nearestNeighborHelper(
    std::shared_ptr<Node> query,
    std::shared_ptr<Node> current,
    int depth,
    double &bestDist,
    std::shared_ptr<Node> bestNode,
    const std::unordered_set<std::shared_ptr<Node>> &exclude)
{
    if (!current) return bestNode;

    // 计算当前节点与查询点的距离（排除自身或已排除节点）
    int axis = depth % 3; // 若支持任意维度，可使用成员变量 dimension_
    double d = (query->pos - current->pos).norm();
    if (current == query || exclude.count(current))
        d = std::numeric_limits<double>::infinity();

    // 更新最优候选
    if (d < bestDist) {
        bestDist = d;
        bestNode = current;
    }

    // 根据当前轴决定“好侧”（near）和“坏侧”（far）
    std::shared_ptr<Node> nearBranch = nullptr, farBranch = nullptr;
    if (query->pos[axis] < current->pos[axis]) {
        nearBranch = current->left;
        farBranch  = current->right;
    } else {
        nearBranch = current->right;
        farBranch  = current->left;
    }

    // 先递归搜索好侧
    bestNode = nearestNeighborHelper(query, nearBranch, depth + 1, bestDist, bestNode, exclude);

    // 如果超平面距离小于当前最小距离，则可能有更近的点，搜索坏侧
    if (farBranch && std::abs(query->pos[axis] - current->pos[axis]) < bestDist) {
        bestNode = nearestNeighborHelper(query, farBranch, depth + 1, bestDist, bestNode, exclude);
    }
    return bestNode;
}

// 修改后的 nearestNeighbor：构造排除集合，并调用辅助函数
std::shared_ptr<Node> KDTree::nearestNeighbor(
    std::shared_ptr<Node> query,
    std::shared_ptr<Node> rootNode, 
    std::shared_ptr<Node> bestNode,
    int depth)
{
    if (!rootNode)
        rootNode = this->root_;
    if (!rootNode)
        return bestNode; // 空树

    // 构造排除集合：将成员级的 notTargetTemp_ 与 notTargetPerm_ 合并
    std::unordered_set<std::shared_ptr<Node>> exclude = this->notTargetPerm_;
    exclude.insert(this->notTargetTemp_.begin(), this->notTargetTemp_.end());

    double bestDist = std::numeric_limits<double>::infinity();
    return nearestNeighborHelper(query, rootNode, depth, bestDist, bestNode, exclude);
}

// 修改后的 kNearestNeighbor：每次调用 nearestNeighbor 时将已找到的节点排除
std::vector<std::shared_ptr<Node>> KDTree::kNearestNeighbor(std::shared_ptr<Node> query, int num)
{
    if (!query)
        throw std::invalid_argument("Input node cannot be null");

    std::vector<std::shared_ptr<Node>> knn;
    // 不能返回查询点本身，所以最多返回 size_ - 1 个邻居
    num = std::min(this->size_ - 1, num);

    // 局部保存已排除的节点，避免影响全局状态
    std::unordered_set<std::shared_ptr<Node>> localExclude = this->notTargetPerm_;
    // 清空临时排除集合（确保每次从空状态开始）
    this->notTargetTemp_.clear();

    for (int i = 0; i < num; ++i) {
        // 更新成员级临时排除集合，以便 nearestNeighbor 在内部构造排除集合时能排除已经找到的邻居
        this->notTargetTemp_ = localExclude;

        auto nearest = nearestNeighbor(query, nullptr, nullptr, 0);
        if (!nearest) {
            this->notTargetTemp_.clear();
            throw std::runtime_error("Failed to find neighbor at iteration " + std::to_string(i));
        }
        knn.push_back(nearest);
        localExclude.insert(nearest);
    }
    this->notTargetTemp_.clear();
    return knn;
}
```

Errors again:
```cpp
Get Roadmap Service Failed: service [/fg_drm/get_roadmap] responded with an error:



[KD-Tree]: Size = 168 Height = 15
[Astar]: No valid path.
[KD-Tree]: Size = 170 Height = 15
[ERROR] [1741442058.877326831, 36718.058000000]: [fgDRMManager] Exception in getRoadmapServiceCB: Failed to find neighbor at iteration 0
terminate called after throwing an instance of 'std::runtime_error'
  what():  Start node is null

Thread 1 "fg_drm_node" received signal SIGABRT, Aborted.
__GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
50	../sysdeps/unix/sysv/linux/raise.c: No such file or directory.
(gdb) bt full
#0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
        set = {__val = {4096, 73014444033, 140737488337968, 3, 0, 0, 7959393510860551283, 
            8246765328066767220, 140737471607407, 140737488344624, 140737488344632, 140737488344640, 
            140737488344648, 0, 140737488344664, 0}}
        pid = <optimized out>
        tid = <optimized out>
        ret = <optimized out>
#1  0x00007ffff791d859 in __GI_abort () at abort.c:79
        save_stage = 1
        act = {__sigaction_handler = {sa_handler = 0x7ffff7ae85c0 <_IO_2_1_stderr_>, 
            sa_sigaction = 0x7ffff7ae85c0 <_IO_2_1_stderr_>}, sa_mask = {__val = {140737348797888, 1, 
              140737348798019, 3432, 140737347369297, 18, 10, 140737348797888, 140737145361984, 
              140737349723008, 140737488340432, 140737488340384, 140737347370643, 10, 140737348797888, 
              140737145361984}}, sa_flags = -138725980, sa_restorer = 0x7ffff7ae8780 <stderr>}
        sigs = {__val = {32, 0 <repeats 15 times>}}
#2  0x00007ffff7ba88d1 in ?? () from /lib/x86_64-linux-gnu/libstdc++.so.6
No symbol table info available.
#3  0x00007ffff7bb437c in ?? () from /lib/x86_64-linux-gnu/libstdc++.so.6
No symbol table info available.
#4  0x00007ffff7bb43e7 in std::terminate() () from /lib/x86_64-linux-gnu/libstdc++.so.6
No symbol table info available.
#5  0x00007ffff7bb4699 in __cxa_throw () from /lib/x86_64-linux-gnu/libstdc++.so.6
No symbol table info available.
#6  0x00007ffff739d315 in PRM::AStar (
    roadmap=std::shared_ptr<class PRM::KDTree> (use count 1, weak count 0) = {...}, 
    start=std::shared_ptr<struct PRM::Node> (empty) = {...}, 
    goal=std::shared_ptr<struct PRM::Node> (empty) = {...}, map=warning: RTTI symbol not found for class 'std::_Sp_counted_ptr<mapManager::occMap*, (__gnu_cxx::_Lock_policy)2>'
warning: RTTI symbol not found for class 'std::_Sp_counted_ptr<mapManager::occMap*, (__gnu_cxx::_Lock_policy)2>'

std::shared_ptr<class mapManager::occMap> (use count 2, weak count 0) = {...})
    at /home/lhx/coding-projects/decision-roadmap/ros/src/global_planner/include/global_planner/PRMAstar.h:24
        path = std::vector of length 0, capacity 288115819723563508
        open = std::priority_queue wrapping: std::vector of length -8796063664641, capacity -2931982080781 = {std::shared_ptr<PRM::Node> (use count -1264850922, weak count -1071483682) = {
            get() = 0x24000000aa}, 
          <error reading variable: Cannot access memory at address 0x7ff0000000000008>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (expired, weak count 0) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x1d}, 
          <error reading variable: Cannot access memory at address 0x243f800008>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x1d}, 
          <error reading variable: Cannot access memory at address 0x25>, 
          <error reading variable: Cannot access memory at address 0x20>, 
          <error reading variable: Cannot access memory at address 0x25>, 
          <error reading variable: Cannot access memory at address 0xcd>, 
          std::shared_ptr<PRM::Node> (use count 458759, weak count 458758) = {get() = 0x7fffe4004a30}, 
          <error reading variable: Cannot access memory at address 0x9e00000051>, 
          <error reading variable: Cannot access memory at address 0xa000000051>, 
          <error reading variable: Cannot access memory at address 0xa200000051>, 
          <error reading variable: Cannot access memory at address 0xa400000052>, 
          <error reading variable: Cannot access memory at address 0xa400000054>, 
          <error reading variable: Cannot access memory at address 0xa300000055>, 
          <error reading variable: Cannot access memory at address 0xa100000055>, 
          <error reading variable: Cannot access memory at address 0x9f00000054>, 
          <error reading variable: Cannot access memory at address 0x9d00000054>, 
          <error reading variable: Cannot access memory at address 0x9b00000055>, 
          <error reading variable: Cannot access memory at address 0x17d>, 
          <error reading variable: Cannot access memory at address 0x100000009>, 
          <error reading variable: Cannot access memory at address 0x9>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x555555aef100}, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x200000000}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, 
          <error reading variable: Cannot access memory at address 0x200000008>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x200000000}, 
          <error reading variable: Cannot access memory at address 0x4d>, 
          std::shared_ptr<PRM::Node> (expired, weak count 0) = {get() = 0x7fffe4005140}, 
          <error reading variable: Cannot access memory at address 0x2c>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x7fffe4136000}, 
          <error reading variable: Cannot access memory at address 0x12d>, 
          std::shared_ptr<PRM::Node> (use count 262148, weak count 65541) = {get() = 0x0}, 
          <error reading variable: Cannot access memory at address 0xd>, 
          <error reading variable: Cannot access memory at address 0x11>, 
          <error reading variable: Cannot access memory at address 0x7c>, 
          <error reading variable: Cannot access memory at address 0x11>, 
          <error reading variable: Cannot access memory at address 0x39>, 
          <error reading variable: Cannot access memory at address 0x15>, 
          <error reading variable: Cannot access memory at address 0x37372e303d>, 
          <error reading variable: Cannot access memory at address 0x16>, 
          <error reading variable: Cannot access memory at address 0x3930312e303d>, 
          <error reading variable: Cannot access memory at address 0xb>, 
          <error reading variable: Cannot access memory at address 0x3930312e303d>, 
          <error reading variable: Cannot access memory at address 0x24>, 
          <error reading variable: Cannot access memory at address 0x2bbe400a308>, 
          <error reading variable: Cannot access memory at address 0x28>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x20}, 
          <error reading variable: Cannot access memory at address 0x2bb00000008>, 
          <error reading variable: Cannot access memory at address 0xdd>, 
          std::shared_ptr<PRM::Node> (use count 262148, weak count 65541) = {get() = 0x7fffe400b7f0}, 
          <error reading variable: Cannot access memory at address 0x1a33633508>, warning: RTTI symbol not found for class 'boost::detail::sp_counted_impl_pd<ros::SubscriptionCallbackHelperT<boost::shared_ptr<rosgraph_msgs::Clock_<std::allocator<void> > const> const&, void>*, boost::detail::sp_ms_deleter<ros::SubscriptionCallbackHelperT<boost::shared_ptr<rosgraph_msgs::Clock_<std::allocator<void> > const> const&, void> > >'
warning: RTTI symbol not found for class 'boost::detail::sp_counted_impl_pd<ros::SubscriptionCallbackHelperT<boost::shared_ptr<rosgraph_msgs::Clock_<std::allocator<void> > const> const&, void>*, boost::detail::sp_ms_deleter<ros::SubscriptionCallbackHelperT<boost::shared_ptr<rosgraph_msgs::Clock_<std::allocator<void> > const> const&, void> > >'

std::shared_ptr<PRM::Node> (use count 1, weak count 0) = {get() = 0x555555ad3b90}, 
          std::shared_ptr<PRM::Node> (expired, weak count 0) = {get() = 0x7fffe400c930}, 
          std::shared_ptr<PRM::Node> (use count -469701072, weak count 32766) = {get() = 0x8}, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (expired, weak count 0) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x7fffdc001080}, 
          <error reading variable: Cannot access memory at address 0x100000007>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, <error reading variable: Cannot access memory at address 0x65646f76>, 
          <error reading variable: Cannot access memory at address 0x1fd>, 
          std::shared_ptr<PRM::Node> (use count 262148, weak count 65541) = {get() = 0x0}, 
          <error reading variable: Cannot access memory at address 0x10ce4009808>, 
          std::shared_ptr<PRM::Node> (use count -469759792, weak count 32766) = {
            get() = 0x7fffe400a560}, 
          std::shared_ptr<PRM::Node> (use count 542608988, weak count 1074752121) = {get() = 0x101}, warning: RTTI symbol not found for class 'ros::Transport'
warning: RTTI symbol not found for class 'ros::Transport'

std::shared_ptr<PRM::Node> (use count -469698080, weak count 32766) = {get() = 0x7fffe4268e10}, 
          std::shared_ptr<PRM::Node> (use count -527561, weak count -536870786) = {
            get() = 0x7fffe400f9c0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x7ffff7f31390 <ros::ConnectionManager::onConnectionHeaderReceived(boost::shared_ptr<ros::Connection> const&, ros::Header const&)>}, std::shared_ptr<PRM::Node> (expired, weak count 0) = {
            get() = 0x555555acf920}, 
          <error reading variable: Cannot access memory at address 0x400000008>, 
          std::shared_ptr<PRM::Node> (use count -2092412075, weak count 1214535915) = {
            get() = 0x7ffff7fc0071 <boost::function4<void, boost::shared_ptr<ros::Connection> const&, boost::shared_array<unsigned char> const&, unsigned int, bool>::assign_to<boost::_bi::bind_t<void, boost::_mfi::mf4<void, ros::ServiceClientLink, boost::shared_ptr<ros::Connection> const&, boost::shared_array<unsigned char> const&, unsigned int, bool>, boost::_bi::list5<boost::_bi::value<ros::ServiceClientLink*>, boost::arg<1>, boost::arg<2>, boost::arg<3>, boost::arg<4> > > >(boost::_bi::bind_t<void, boost::_mfi::mf4<void, ros::ServiceClientLink, boost::shared_ptr<ros::Connection> const&, boost::shared_array<unsigned char> const&, unsigned int, bool>, boost::_bi::list5<boost::_bi::value<ros::ServiceClientLink*>, boost::arg<1>, boost::arg<2>, boost::arg<3>, boost::arg<4> > >)::stored_vtable+1>}, 
          std::shared_ptr<PRM::Node> (use count 4, weak count -1) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0xffffffff}, <error reading variable: Cannot access memory at address 0x100000109>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x7ffff7ed2620 <ros::Connection::onHeaderWritten(boost::shared_ptr<ros::Connection> const&)>}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x7fffe400a560}, 
          <error reading variable: Cannot access memory at address 0x100000007>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0xffffffff}, 
          <error reading variable: Cannot access memory at address 0x5603df08>, 
          std::shared_ptr<PRM::Node> (use count 1224444232, weak count 1213262978) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (use count 4, weak count -1) = {get() = 0x0}, 
          std::shared_ptr<PRM::Node> (use count 33, weak count -1) = {
            get() = 0x7ffff7fbe048 <vtable for boost::signals2::signal<void (boost::shared_ptr<ros::Connection> const&, ros::Connection::DropReason), boost::signals2::optional_last_value<void>, int, std::less<int>, boost::function<void (boost::shared_ptr<ros::Connection> const&, ros::Connection::DropReason)>, boost::function<void (boost::signals2::connection const&, boost::shared_ptr<ros::Connection> const&, ros::Connection::DropReason)>, boost::signals2::mutex>+16>}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x7fffe400e1e0}, 
          <error reading variable: Cannot access memory at address 0x100000007>, 
          std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {
            get() = 0x7ffff7f33500 <ros::ConnectionManager::tcprosAcceptConnection(boost::shared_ptr<ros::TransportTCP> const&)+944>}, <error reading variable: Cannot access memory at address 0xed>, 
          std::shared_ptr<PRM::Node> (use count 262148, weak count 65541) = {get() = 0x7fffe400a840}, 
          <error reading variable: Cannot access memory at address 0x2765646f6e67>, 
          <error reading variable: Cannot access memory at address 0x383332613433336d>, 
          <error reading variable: Cannot access memory at address 0x363961303431626b>, 
          <error reading variable: Cannot access memory at address 0x7571657200000035>, 
          <error reading variable: Cannot access memory at address 0x5f6c61626f6c6745>, 
          <error reading variable: Cannot access memory at address 0x6d64616f5274654f>, 
          <error reading variable: Cannot access memory at address 0x7365720000002f7c>, 
          <error reading variable: Cannot access memory at address 0x61626f6c673d6578>, 
          <error reading variable: Cannot access memory at address 0x616f527465472f7a>, 
          <error reading variable: Cannot access memory at address 0x1e65736e77>, <error reading variable: Cannot access memory at address 0x6e616c705f6c616a>, <error reading variable: Cannot access memory at address 0x736f70616d646177>, <error reading variable: Cannot access memory at address 0x39>, std::shared_ptr<PRM::Node> (use count 49, weak count -1) = {get() = 0x7fffe4059f80}, <error reading variable: Cannot access memory at address 0x8f6ee4018608>, <error reading variable: Cannot access memory at address 0xec>, std::shared_ptr<PRM::Node> (use count 262148, weak count 65541) = {get() = 0x7fffe4029350}, <error reading variable: Cannot access memory at address 0x35646d000000276d>, <error reading variable: Cannot access memory at address 0x3931646238333269>, <error reading variable: Cannot access memory at address 0x3437636636396138>, <error reading variable: Cannot access memory at address 0x5f7473657571657a>, <error reading variable: Cannot access memory at address 0x6e616c705f6c616a>, <error reading variable: Cannot access memory at address 0x655270616d646177>, <error reading variable: Cannot access memory at address 0x736e6f7073657208>, <error reading variable: Cannot access memory at address 0x6c705f6c61626f74>, <error reading variable: Cannot access memory at address 0x70616d64616f527c>, <error reading variable: Cannot access memory at address 0x6570797400000026>, <error reading variable: Cannot access memory at address 0x2f72656e6e616c78>, <error reading variable: Cannot access memory at address 0x736f70616d647069>, <error reading variable: Cannot access memory at address 0x39>, std::shared_ptr<PRM::Node> (use count 49, weak count -1) = {get() = 0x7fffe40297e0}, <error reading variable: Cannot access memory at address 0x8f6e33633508>, <error reading variable: Cannot access memory at address 0x2c>, std::shared_ptr<PRM::Node> (use count -469759792, weak count 32766) = {get() = 0x7fffe405af20}, <error reading variable: Cannot access memory at address 0x17d>, <error reading variable: Cannot access memory at address 0x100000009>, <error reading variable: Cannot access memory at address 0x9>, std::shared_ptr<PRM::Node> (empty) = {get() = 0x55556cd04490}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x200000000}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, <error reading variable: Cannot access memory at address 0x200000008>, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x200000000}, <error reading variable: Cannot access memory at address 0x1ed>, std::shared_ptr<PRM::Node> (use count 1293080650, weak count 1074340346) = {get() = 0x555555aefc40}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (use count 2043552312, weak count 1072879995) = {get() = 0x555555fa1600}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x555555c4d4c0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x555555bd8d20}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x555556091210}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x555556076b30}, std::shared_ptr<PRM::Node> (use count -990055245, weak count 1073825627) = {get() = 0x0}, std::shared_ptr<PRM::Node> (use count 542608988, weak count 1073703545) = {get() = 0x555555c80ec0}, std::shared_ptr<PRM::Node> (use count 542608988, weak count 1072654969) = {get() = 0x555555adc590}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x5555560c3bf0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x555555c7ef80}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x555555bd8cc0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (use count 542608988, weak count 1074752121) = {get() = 0x0}, std::shared_ptr<PRM::Node> (use count 1293080650, weak count 1072243194) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x55555603b1a8}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x555555bd88f0}, std::shared_ptr<PRM::Node> (empty) = {get() = 0x555556043b80}...}
        close = std::unordered_set with 93825388856560 elements = {[0] = <error reading variable: Cannot access memory at address 0x3ff118bdc064f6bd><error reading variable: Cannot access memory at address 0xc02cc1f285363984>...}
        record = std::vector of length 8796063665696, capacity 259868801 = {<error reading variable record (Cannot access memory at address 0x0)>
        findPath = <optimized out>
        ptr = std::shared_ptr<PRM::Node> (use count 10277, weak count -1769664769) = {get() = 0x7ffff7dc0c10 <__GI___pthread_key_create>}
#7  0x00007ffff73c9109 in globalPlanner::FGDRM::waypointUpdateCB (this=0x555555ae4f10) at /home/lhx/coding-projects/decision-roadmap/ros/src/global_planner/include/global_planner/fg_drm.cpp:1015
        temp_goal = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}
        lastNavWaypoint = {x = -3.76360583, y = -4.76329994, z = 0.83878845, intensity = 0}
        currPos = std::shared_ptr<PRM::Node> (use count 1, weak count 0) = {get() = 0x55556d0f7220}
        start = std::shared_ptr<PRM::Node> (empty) = {get() = 0x0}
        path = std::vector of length 0, capacity 288115819723563508
        waypoint = {header = {seq = 1, stamp = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 4154294453, static MIN = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 1, static MIN = <same as static member of an already seen type>, static MAX = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 4294967295, nsec = 999999999, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static MAX = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 4294967295, nsec = 999999999, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static ZERO = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, static UNINITIALIZED = {<ros::TimeBase<ros::Time, ros::Duration>> = {sec = 0, nsec = 0, static MIN = <same as static member of an already seen type>, static MAX = <same as static member of an already seen type>, static ZERO = <same as static member of an already seen type>, static UNINITIALIZED = <same as static member of an already seen type>}, <No data fields>}}, <No data fields>}, frame_id = ""}, point = {x = 3.715800035378535e-317, y = 4.552282096390673e-315, z = 6.9533160207957377e-310}}
        lock = {_M_device = @0x555555ae55e8}
        __PRETTY_FUNCTION__ = "void globalPlanner::FGDRM::waypointUpdateCB(const ros::TimerEvent&)"     
```