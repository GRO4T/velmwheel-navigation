import numpy as np
import tensorflow as tf
from tensorflow.keras import Sequential, layers, losses
from tensorflow.keras.models import Model
from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.data import Dataset
import sys
import os
from datetime import datetime
import matplotlib.pyplot as plt

BATCH_SIZE = 32
EPOCHS = 100

class Autoencoder(Model):
    """docstring for Autoencoder"""
    def __init__(self, latent_dim = 8, input_size = 26, enc_layers = [], dec_layers = []):
        super(Autoencoder, self).__init__()
        self.latent_dim = latent_dim
        self.input_size = input_size
        self.encoder = Sequential(
            [layers.Input(shape = (self.input_size))] + 
            [layers.Dense(size, activation = 'relu') for size in enc_layers] + 
            [layers.Dense(self.latent_dim, activation = 'relu')])
        self.decoder = Sequential(
            [layers.Input(shape = (self.latent_dim))] + 
            [layers.Dense(size, activation = 'relu') for size in dec_layers] + 
            [layers.Dense(self.input_size, activation = 'tanh')])
        self.encoder.summary()
        self.decoder.summary()

    def call(self, x):
        return self.decoder(self.encoder(x))

def main():
    train_ds = None
    for ds_file in os.listdir(sys.argv[1]):
        new_ds = Dataset.load(sys.argv[1] + '/' + ds_file)
        if train_ds == None:
            train_ds = new_ds
        else:
            train_ds = train_ds.concatenate(new_ds)
        print(f'Loading file: {ds_file}')
        print(f'New dataset size: {len(list(train_ds))}')

    # remove empty angle values from imu
    train_ds = train_ds.map(lambda x: tf.concat([x[:20], x[21:]], 0))

    # calculate absolute maximum for each column
    maximum = np.absolute(np.array(list(train_ds))).max(axis = 0)
    print(f'Absolute maximum: {maximum}')

    # prepare dataset - normalize, group,batch
    train_ds = train_ds.map(lambda x: x/maximum)
    train_ds = train_ds.map(lambda x: (x, x))
    train_ds = train_ds.batch(BATCH_SIZE).prefetch(1)


    for LATENT_DIM in range(6, 14):
        for ENC_LAYERS in [[], [16], [32, 16], [64, 32, 16]]:
            # LATENT_DIM = 8
            # ENC_LAYERS = []
            DEC_LAYERS = ENC_LAYERS[::-1]
            output_folder = datetime.now().strftime(f'odometry_anomaly_detector/models/autoencoder/{len(list(train_ds))}_E{EPOCHS}_B{BATCH_SIZE}_L{LATENT_DIM}_E{ENC_LAYERS}_D{DEC_LAYERS}_%Y%m%d_%H%M%S/')
            autoencoder = Autoencoder(latent_dim = LATENT_DIM, enc_layers = ENC_LAYERS, dec_layers = DEC_LAYERS)
            autoencoder.compile(optimizer = 'adam', loss = losses.MeanSquaredError())
            history = autoencoder.fit(train_ds, batch_size = BATCH_SIZE, epochs = EPOCHS, shuffle = True)#, callbacks = [ModelCheckpoint(output_folder + 'save_{epoch}', save_weights_only = True)])

            plt.figure()
            plt.plot(history.history['loss'], label = 'loss')
            plt.title(f'Loss after every epoch (lowest loss: {min(history.history["loss"]):.5f})')
            os.makedirs(output_folder)
            plt.savefig(output_folder + 'history.jpg')
            # plt.show()

if __name__ == '__main__':
    main()