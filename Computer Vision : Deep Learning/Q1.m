
% This code will run much faster if you install
function layers = Q1()

    % ** Modify the code below **
    layers = [
            imageInputLayer([32 32 3]); % input images are 32x32x3

            % Use the following structural building blocks to answer the
            % question.
            % In particular, please use a 'BiasLearnRateFactor' of 2.
            % 
%             convolution2dLayer(kernelSize,numChannels,'Padding',2,'BiasLearnRateFactor',2);
%             maxPooling2dLayer(kernelSize,'Stride',2);
%             reluLayer();
%             batchNormalizationLayer;
%             averagePooling2dLayer(kernelSize,'Stride',2);
%             fullyConnectedLayer(numUnits,'BiasLearnRateFactor',2);
            
            % YOUR CODE HERE
            convolution2dLayer(5,16,'Padding',2,'BiasLearnRateFactor',2);
            maxPooling2dLayer(3,'Stride',2);
            reluLayer();
            
            convolution2dLayer(5,32,'Padding',2,'BiasLearnRateFactor',2);
            batchNormalizationLayer;
            averagePooling2dLayer(3,'Stride',2);
            reluLayer();
            
            convolution2dLayer(5,64,'Padding',2,'BiasLearnRateFactor',2);
            batchNormalizationLayer;
            averagePooling2dLayer(3,'Stride',2);
            reluLayer();
            
            fullyConnectedLayer(64,'BiasLearnRateFactor',2);
            reluLayer();
            
            fullyConnectedLayer(6,'BiasLearnRateFactor',2);


            softmaxLayer();
            classificationLayer();];
    
end
