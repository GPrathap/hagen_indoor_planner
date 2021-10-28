import gym
import numpy as np

class Exercise01:
    def __init__(self, env, hidden=1):
        self.pvariance = 0.1     # variance of initial parameters
        self.ppvariance = 0.02   # variance of perturbations
        self.nhiddens = hidden       # number of internal neurons
        # the number of inputs and outputs depends on the problem
        # we assume that observations consist of vectors of continuous value
        # and that actions can be vectors of continuous values or discrete actions
        self.env = env
        self.ninputs = self.env.observation_space.shape[0]
        if (isinstance(self.env.action_space, gym.spaces.box.Box)):
            self.noutputs = self.env.action_space.shape[0]
        else:
            self.noutputs = self.env.action_space.n
        self.rewards = []

    def init_network(self):
        # initialize the training parameters randomly by using a gaussian
        # distribution with average 0.0 and variance 0.1
        # biases (thresholds) are initialized to 0.0
        self.W1 = np.random.randn(self.nhiddens, self.ninputs) * self.pvariance      # first connection layer
        self.W2 = np.random.randn(self.noutputs, self.nhiddens) * self.pvariance    # second connection layer
        self.b1 = np.zeros(shape=(self.nhiddens, 1))                      # bias internal neurons
        self.b2 = np.zeros(shape=(self.noutputs, 1)) 

    def network_update(self, observation):
        # observation = env.reset()
        # convert the observation array into a matrix with 1 column and ninputs rows
        # observation = observation.resize(ninputs, 1)
        # compute the netinput of the first layer of neurons
        Z1 = np.dot(self.W1, observation) + self.b1
        # compute the activation of the first layer of neurons with the tanh function
        A1 = np.tanh(Z1)
        # compute the netinput of the second layer of neurons
        Z2 = np.dot(self.W2, A1) + self.b2
        # compute the activation of the second layer of neurons with the tanh function
        A2 = np.tanh(Z2)
        # if the action is discrete
        return A2
    
    def evaluate(self, episodes=20, steps=20):
        for i_episode in range(episodes):
                observation = env.reset()
                rewards = 0
                for t in range(steps):
                    # env.render()
                    A2 = self.network_update(observation)
                    #  select the action that corresponds to the most activated unit
                    if (isinstance(env.action_space, gym.spaces.box.Box)):
                        action = A2
                    else:
                        action = np.argmax(A2)
                    observation, reward, done, info = env.step(action)
                    rewards += reward
                    # print(reward, rewards)
                    if done:
                        print("Episode finished after {} timesteps".format(t+1))
                        break
                self.rewards.append((i_episode, rewards))
        env.close()
        self.rewards = np.array(self.rewards)


env = gym.make('CartPole-v0')
exe01 = Exercise01(env, hidden=1)
exe01.init_network()
exe01.evaluate(episodes=10, steps=20)
print("Rewards", exe01.rewards)
