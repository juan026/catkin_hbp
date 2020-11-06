import torchaudio
import torch
import librosa
import numpy as np

from torchvision.models import mobilenet_v2


class AudioClassifier:

    def __init__(self, checkpoint, class_id_map, num_clases, device='cpu'):
        
        self.class_id_map = class_id_map

        self.device = torch.device(device)

        # define model topoly
        self.audio_classifier_model = mobilenet_v2()
        # set up input conv shape and classifier output
        self.audio_classifier_model.classifier[1] = torch.nn.Linear(in_features=1280,
                                        out_features=num_clases, bias=True)
        self.audio_classifier_model.features[0][0] = torch.nn.Conv2d(1, 32, 
                                        kernel_size=(7, 7), stride=(2, 2), 
                                        padding=(3, 3), bias=False)
        
        # load pretrained checkpoint
        state_dict = torch.load(checkpoint, map_location=device)
        self.audio_classifier_model.load_state_dict(state_dict)
        self.audio_classifier_model.to(device)

        # set model for evaluation
        self.audio_classifier_model.eval()

    def get_class_id_map(self):
        return self.class_id_map

    def get_class_label(self, class_id):

        return self.class_id_map[class_id]


    def run_inference(self, wav_file_path):
        # read wav file in 16k sr
        audio_data, sample_rate = librosa.load(wav_file_path, 
                                                sr=16000)
        
        # normalize the time signal
        audio = (np.float32((audio_data-np.mean(audio_data)) 
                    / np.std(audio_data) )+ 1e-15)
        
        # compute mel spectrogram
        audio = torch.tensor(audio)
        spect = torchaudio.transforms.MelSpectrogram(n_fft=320, 
                              sample_rate=sample_rate)(audio[:])

        # reshape mel spectrogram
        spect = spect.view(1, 1, spect.shape[0], spect.shape[1])

        # run inference
        res = self.audio_classifier_model(spect)
        softmax = torch.nn.Softmax(dim=1)
        
        # return detected class
        return np.argmax(softmax(res).detach().numpy())


if __name__ == '__main__':


    # testing model inference
    class_id_map = {0:'cat',
           1:'dog',
           2:'bike',
           3:'car'}

    classifier = AudioClassifier(checkpoint="../models/audio_classifier_mobilenet_v2_chkpt.pth.tar", 
    class_id_map=class_id_map, num_clases=4)

    class_id = classifier.run_inference("../../audio_files/bicycle_bell.wav")

    print(classifier.get_class_label(class_id))
    class_id = classifier.run_inference("../../audio_files/car_horn.wav")

    print(classifier.get_class_label(class_id))
    class_id = classifier.run_inference("../../audio_files/cat_meow.wav")

    print(classifier.get_class_label(class_id))
    class_id = classifier.run_inference("../../audio_files/dog_bark.wav")

    print(classifier.get_class_label(class_id))