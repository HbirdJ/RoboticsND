[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# Deep Learning Project #

In this project, I trained a deep neural network to identify and track a target in simulation. So-called “follow me” applications like this are key to many fields of robotics and the very same techniques you apply here could be extended to scenarios like advanced cruise control in autonomous vehicles or human-robot collaboration in industry.

[image_0]: ./docs/misc/sim_screenshot.png
[diagram]: ./data/figures/shape.png
[result]: ./data/figures/example_tuples.png
[failures]: ./data/figures/failed_nets.png

![alt text][image_0] 

## FCN Brief Overview
Based on the project guidelines, a Fully Convolutional Network is used. This allows pixel specific analysis that can be carried through the entire network. The input image goes through an encoder step using convolutional 2D layers which allows a 1x1 convolution layer to be used on a smaller input.

This condensed image then goes through decoder blocks that upsample the data back to its original size. The final output image weights three different classes: target, people, and neither. 

## Neural Network Architecture

Using the functions built in the semantic segmentation lab, the following fully convolutional network was built:

```
    # Encoder Blocks
    encoder1 = encoder_block(inputs, 16, stride)
    encoder2 = encoder_block(encoder1, 32, stride)
    encoder3 = encoder_block(encoder2, 64, stride)
    
    # Dropout added
    encoder3 = Dropout(keep_prob)(encoder3)

    # 1x1 Convolution layer using conv2d_batchnorm().
    onebyone = conv2d_batchnorm(encoder3, 128, kernel_size=1, strides=1)
    
    # Decoder Blocks
    decode1 = decoder_block(onebyone, encoder2, 64)
    decode2 = decoder_block(decode1, encoder1, 32)
    decode3 = decoder_block(decode2, inputs, 16)
```

This diagram further explains the network geometry:

![Network Diagram][diagram]

Each 2D convolutional layer is connected to a matching layer across the 1x1 layer.

### Skip Connections
Skip connections are an effective way to transfer positional data across the network. Each encoder layer is linked to its corresponding decoder layer. The decoder block function concatenates the matching encoder block output to tie the data together. Without these connections the positional learnings of the initial encoder layers would not transition.

### Dropout
Dropout refers to the neural net systematically ignoring random weights during the training phase. This helps prevent the model from overfitting to the training data. Ignoring random neurons during the training phase helps prevent the network from relying on given signifiers in the training data.

Implementing dropout for this dataset proved crucial to improve the model's performance. A dropout probability of .4 was used. An initial value of .5 was used, but the model had trouble converging.

## Parameter Choices

### Epochs
The epoch count is how many times the full training set passes through the network. Varying batch sizes and steps per epoch determine how large this full training set is. Choosing the number of epochs is done by observing network performance.

While training on the final model, a run of 15 epochs would be run, the outputs observed, and then additional sets of 15 were run if results were still improving. In total, 45 epochs were run to generate the final model.

Using an AWS instance with a dedicated GPU meant each epoch took approximately 51 seconds to run.

### Batch Size 
Batch size is limited by the CPU. A larger batch size of 128 appeared to slow down on even the AWS instance, so even 64 was used for the final model.

### Steps Per Epoch
With the given set of 4131 training images, 64 steps were run per epoch. This ensured all images would make it through the network on each epoch.

### Learning Rate
Choosing the correct learning rate means balancing between a model that changes too quickly or not at all. A high learning rate will never converge on a useful solution because the weights are quick to change. A low learning rate will take much longer to find a solution and is much more likely to become stuck at a local minimum or maximum.

In this network a learning rate of 0.01 was found to be effective.

## Results
Through the use of an Amazon AWS instance, over 20 models were trained and tested. Different architecture was tested alongside tweaking of the different parameters. While a few networks produced final scores of zero, most test models trained produced scores between .2 and .37 until the final model produced a successful .402.

The following image shows the final output of the model in the right column. The left column is the input image, and the middle is the expected output. The model is most successful when following the target when it's in the center of the screen.

![Example Model][results]

The next image is a compilation of some of the models that failed to reach the .4 goal. The biggest challenge most faced was identifying the target when it was in the distance.

![Failed models][failures]

## Summary
This model is effective at helping the simulated drone follow its target. A fully convolutional net was required by the given software to accurately allow the drone to make adjustments based on the targets location. This model is also adaptable to other applications where the desire is to partition different pixels into three categories.

For a different target, this model could be used as long as adequate training data could be produced. The target outputs could be merged with the non target results, and new training data with a suitably different target could be implemented without too much additional work.

In reality, if the goal of a network is only to find and follow a target, a simple bounding box model may get the job done while being simpler to build. 

## Model Improvement Possibilities
There's a wide variety of potential ways to improve the performance of this model. The following two options are not comprehensive but offer a start.

* **Collect additional data**: Deep learning is most valuable when there's a huge amount of data avaiable. In this case, a huge chunk of the data set doesn't have the target in it. Gathering additional training data could focus on the model's current weakness of identifying the target when it's far away and in a crowd.
* **Additional tweaking of the network shape**: Experiments with both a shallower and a deeper model appeared to initially offer worse results. Dropping the encoder and decoder down to two blocks from three produced a model with a score of 0.0. While possible to get results with different models, this was outside the scope of this project. Once the given architechture was built and appeared to be giving promising results, no major changes were made outside of modifying the number of filters.