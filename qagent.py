#!/usr/bin/env python
import numpy as np
import random
import csv


class QAgent:

    def __init__(self, alpha = 0.8, gamma = 0.8, epsilon = 0.4, qmatrix = [], seed = 123):
            self.qmatrix = qmatrix
            self.epsilon = epsilon
            self.gamma = gamma
            self.alpha = alpha
            self.seed = seed
            self.rnd = random.seed(self.seed)

    def create_qmatrix(self, states_count, actions_count):
    #Create matrix based on given dimensions
        qm = np.zeros((states_count, actions_count))
        self.qmatrix = qm

    def qmatrix_from_file(self, filename):
    #Load qmatrix from csv file
        self.qmatrix = np.loadtxt(filename,dtype=float,delimiter=',')

    def qmatrix_from_list(self, list):
    #Load qmatrix from list of lists
        self.qmatrix = np.array(list)

    def save_qmatrix (self, filename):
    #Save qmatrix to csv file
        f = open(filename, 'w')
        writer = csv.writer(f)
        writer.writerows(self.qmatrix)
        f.close()

    def chose_action(self, current_state):
    #Chooses the next action based on the current_state.
        this_action = -10000

        if random.random() < self.epsilon:
            possible_actions = self.get_action_ids(current_state)
            this_action = possible_actions[random.randrange(len(possible_actions))]
        else:
            if self.sum_possible_actions(current_state) != 0:
                possible_q_rewards = []
                for possible_action in self.get_action_ids(current_state):
                    possible_q_rewards.append(self.qmatrix[current_state,possible_action])
                this_action =self.get_action_ids(current_state)[np.argmax(possible_q_rewards)]
            else:
                possible_actions = self.get_action_ids(current_state)
                this_action = possible_actions[random.randrange(len(possible_actions))]

        return this_action

    def update_q(self, current_state, action, next_state, reward):
    #Updates the qvalue(current_state, action) using reward, then assigns the next_state value to current_state.

        qsa = self.qmatrix[current_state, action]
        action_values = self.get_action_values(next_state)
        new_q = qsa + self.alpha * (reward + self.gamma * action_values[np.argmax(action_values)] -qsa)
        self.qmatrix[current_state, action] = new_q

    def get_action_ids (self, state):
        ids = []
        for i in range (np.shape(self.qmatrix)[1]):
            if self.qmatrix[state, i] == -10000:
                continue
            ids.append(i)
        return ids

    def sum_possible_actions(self,state):
        action_sum = 0
        acts = self.get_action_ids(state)
        for i in range (len(acts) - 1):
            this_value = self.qmatrix[state, acts[i]]
            if this_value == -10000:
                continue
            action_sum = action_sum + this_value
        return action_sum

    def get_action_values (self,state):
        val = []
        for i in range (np.shape(self.qmatrix)[1]):
            if self.qmatrix[state, i] == -10000:
                continue
            val.append(self.qmatrix[state, i])
        return val