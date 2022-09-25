### ⛑️ Supporting code for https://github.com/robotology/community/discussions/593

### 💫 Run the experiment
Go through the following steps:
1. Click on the badge below to get started with your Gitpod workspace.

   [![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/from-referrer)
   
   Remember to visit the [Gitpod Integrations](https://gitpod.io/integrations) to make sure that all GitHub options are ticked in.
   To find out more about how we use Gitpod, visit our [community](https://github.com/robotology/community/discussions?discussions_q=gitpod).

1. Click on `Ports` (right above the terminal window) and select the port `6080` to display the X window. 
1. Build and run the code from the terminal:
   ```console
   cd smoke-test
   ./test.sh
   ```
   🤖 You will see the iCub pushing the ball.
1. The output of the skin port (downsampled 10 times) can be shown by doing:
   ```console
   gp open /tmp/right_hand/data.log
   ```
   ⚠️ Inspect the whole content of the file as many null values are expected.
