# 节点间数据交换话题命名规则

本文档描述该自瞄项目不同程序之间通过iceoryx通信使用的话题的加载与命名规则

所有程序依赖包含该节点所需的配置信息的文件来初始化，其路径由xmake根目录中定义的task在启动节点时传入，原则上必须是config内与**节点同名的yaml文件**。节点从文件加载的配置信息的键通常是通过统一定义的**枚举反射得到**的，值是通过自定义的yamlcfg对象链式调用得到。所有参数都是**静态的**，只在最外层查询一次后便打包到对应参数结构体里用来构造各个模块。

同时，各个节点的yaml参数文件必须包含指向core.yaml的键值对。core配置文件最顶层键必须是每个节点初始化PoshRuntime时注册的名称（也是节点的程序名称、文件夹名称），每个顶层键下分为四个值为map类型的键值对，其键分别为subtribe、publish、server和client，其值存储的map的键值分别为 由GETNAME反射得到的订阅/发布者的的名称+sub/pub 和 其要订阅/发布的话题的{service, instance, event}，其中要求service是该 发布节点/发布该节点订阅的数据的节点 所在**功能模块的名称**，instance是需要收发的数据的**结构体名称**（当其为服务通信时，则去除掉结构体结尾的request/response后缀），event是具体化这个消息的**目的**（debug、result .etc）

- 所有数据全循序蛇形命名法

````yaml
NodeName:                 # 节点名称，必须与节点程序名称和文件夹名一致
  Subtribe:               # 节点订阅的消息
    SuberName1:
      {ServiceName, InstanceName, EventName}
    SuberName2:
      {ServiceName, InstanceName, EventName}
    ......
  Publish:                # 节点发布的消息
    PuberName:
      {ServiceName, InstanceName, EventName}
    ......
  Server:
  	......
  Client:
  	......
````

---

- update26.1.15: 引入了yaml_cpp_struct通过反射进行参数加载，由于库的局限性无法加载多层yaml，故配置路径改为`config/包名称/组件名称.yaml`

- update26.1.18：不是所有的订阅、服务、话题都要到配置文件里找，像tf、image_raw一类固定的topic就直接硬编码吧
