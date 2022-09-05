# Thesis

## Preparation
1. Install VirtualBox, and then install Ubuntu 16.04 on the VirtualBox. 
2. Install NS3 on Ubuntu and Li-Fi model on NS3.
> For the detailed procedure for the two steps above, please refer to the file [here](https://drive.google.com/drive/folders/1dnc_JtQwPAUOIiE_HMm0tblodgpdpcgR?usp=sharing) or you can find it in the folder `NS3` of the lab server.

3. A video that explains the functions of ns3 simulator can also be found in the folder `NS3` of the lab server. 

## Run Simulation
### Parameter Setting
We can configure parameters through the header file `global_configuration.h`. For example, we can set the total number of UEs to 20 by assigning 20 to the variable `UE_num` defined in `global_configuration.h`. Basically, all simulation parameters listed in the thesis can be found in this header file, and we can set them to any value we want.

In addition, two macros `PROPOSED_METHOD` and `PCSBM` and one variable `complete_config_period` controls which LB method to be execute. The following table shows the corresponding setting for different LB methods:
|  | PROPOSED_METHOD | PCSBM | complete_config_period |
| :-: | :--------------: | :-----: | :--------------: |
| Referenced method | 0 | x | x |
| FCSBM | 1 | 0 | x |
| PCSBM | 1 | 1 | state_num |
| Hybrid | 1 | 1 | one of the values in {-5,-3,2,3,5}| 

> x: don't care

Besides, if we want to discuss the effect of the number of UEs, we have to explicitly declare `UE_num` in `global_configuration.h` as `extern`, and it should be defined in `thesis.cc` as 30 or any initial value we want. Then, wrap the body of the main function in a while-loop that iterates `UE_num` to 180, as shown in the code snippet below.

```cpp=
// global_configuration.h
extern int UE_num;

// thesis.cc
int UE_num = 30;
...
    
int main(int argc, char *argv[])
{
    while (UE_num <= 180) {
        ...
            
        UE_num += 10;
    }
}
```

Likewise, if multiple factors are to be discussed, then use nested loops, one for each factor. For example, if we want to discuss the effect of the number of UEs on different hybrids, then we write the following code.

> In line 7, negative numbers in `period_cand` represents that the hybrids execute FCSBM more with the ratio $(\text{abs}(\text{negative number})-1):1$.
```cpp=
// global_configuration.h
extern int UE_num;
extern int complete_config_period;

//main.cc
int UE_num = 30;
std::vector<int> period_cand = {-5, -3, 2, 3, 5};
int complete_config_period = period_cand.front();

int main(int argc, char *argv[])
{
    for (int i = 0; i < period_cand.size(); i++) {
        complete_config_period = period_cand[i];
        UE_num = 30;
        
        while (UE_num <= 180) {
            ...
                
            UE_num += 10;
        }
    }
}
```

### Output File
Output file will be created in `/home/hsnl/repos/ns-3-allinone/ns-3.25/scratch/thesis/proposed/ (or /benchmark/)`, depending on which LB method now is being executed. Its name is of the form: `Hybrid(UE=XX,demand=(1,XXX),period=X:X)` if hybrid method is executed; otherwise, `method_name(UE=XX,demand=(1,XXX))`.

Besides, if we do the simulation $N$ times, then there are $N$ rows in the output file, each of which has four values: average throughput, average satisfaction, variance of satisfaction and execution time.

### Batch Processing
The shell script `output.sh` in the directory `/home/hsnl/repos/ns-3-allinone/ns-3.25` is used to execute our simulation multiple times.

This is an example of batch file. We can decide the number of times simulation should be conducted by changing the number `1000` to any value we want in line 3.

```shell=
#!/bin/bash

for i in {1,1000}; do
    ./waf --run scratch/thesis/thesis;
    date;
done
```


:::warning
If you fail to run my code, you can check the following several things first:
1. Make sure that the value of the key `directory` of every object in `/home/USER_NAME/repos/ns-3.25/build/compile_commands.json` is equal to the path of `build` directory.
2. Make sure that the output path in `thesis.cc` is updated to match the machine you currently use.
3. If the error like "'>>' should be '> >' within a nested template argument list" appears, then cd to `/home/USER_NAME/repos/ns-3.25/` and enter the following command in terminal to explicitly specify the version of c++ compiler:
```shell=
CXXFLAGS="-Wno-error -std=c++11" ./waf configure
```
:::


## Method Description
No matter which method is executed, they all follow the  execution flow below:

![](https://i.imgur.com/GBji7aK.png =480x)

> $N_s$ is the total number of states in one simulation.
### Referenced Method
- Source code is in `benchmark.cc`.
- `benchmarkDynamicLB()` is the body of the referenced mehtod. The following four subprograms are called in this function, and we are going to discuss them based on their executing order.
    - `precalculation()` is to estimate the channel condition like channel gain, SINR, and calculate the link data rate for each connection between a UE and an AP.
    - `initializedStep()` is to randomly choose an AP from  a UE's strategy set and assign this AP to the UE.
    - Then, keep executing `EGT_basedLoadBalance()` until converges.
    - In the end, `UpdateApAssociationResult()` and `UpdateResourceAllocationResult()` are called to update the results of APA and RA.

### Proposed methods
- Source code is in `proposed_method.cc`.
- `partiallyConfigSatisfactionBalancedMethod()` and `fullyConfigSatisfactionBalancedMethod()` are bodies for PCSBM and FSCBM, respectively. For more comprehensive understanding of each function, please refer to my theis or the comments in the source codes.
