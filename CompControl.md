## ReplayBG Model

The ReplayBG simulation is defined by the following differential equations:

<div>
    <img src="model.png" width="1000"/>
</div>


### Outputs

The output is the blood glucose. This hypoglycemia limit of 70 (mg/dl) is a hard lower-limit for blood glucose, as dropping below 70 (mg/dl) is a significant health risk. In addition, if possible, blood glucose should be kept below 180 (mg/dl). This hyperglycemia limit is a "soft constraint" rather than a hard constraint---minor infractions are not critical, but it is better to keep blood glucose below 180 (mg/dl) if possible.


### Inputs

Two inputs affect glucose levels:
- $i(t)$: Exogenous/Basal insulin injections (units "U" of insulin). Controlled.
- $m(t)$: Carbohydrate intake/"meals" (g). Not controlled.

Thus, the input is two-dimensional: $\textbf{u} = \begin{bmatrix}i(t) & m(t) \end{bmatrix}^T$. In the literature, $i(t)$ is also refered to as I(t) or basal_insulin, and $m(t)$ is also refered to as $CHO(t)$.

While we cannot control the meals, we assume that the meal injections are known by the controller (hence, being an "input" rather than a disturbance). 

The meals are determined by the `data_cho.csv` file and are parsed by the get_discretized_meals() function, which multiplies the values in the data.csv file by 5 because each meal takes 5 minuntes. 

The controller determines the exogenous insulin injection in control time intervals that are specified by the user (e.g. 5 or 10 minutes). The control time interval is a design decision for you to make. 



### Insulin Limits

The maximum exogenous/basal insulin injection $i(t)$ is 0.04, and the minimum is 0.


### Default Meal Values

The default meal values, which InsulinCo is interested in with the initial (model-based and data-driven) control designs, are 35 g for breakfast and 45 g for lunch and dinner.

For the bonus, you might consider larger/different meals.


### State Variables
The state of the system is defined by the following variables:
- $G_p(t)$: Plasma glucose concentration (mg/dl);
- $X(t)$: Insulin action on glucose disposal and production ($min^{-1}$);
- $Q_{sto1}(t)$: Amount of glucose in the stomach in solid state (mg/kg);
- $Q_{sto2}(t)$: Amount of glucose in the stomach in liquid state (mg/kg);
- $Q_{gut}(t)$: Glucose concentration in the intestine (mg/kg);
- $I_{sc1}(t)$: Insulin in a non-monomeric state (mU/kg);
- $I_{sc2}(t)$: Insulin in a monomeric state (mU/kg);
- $I_p(t)$: Plasma insulin concentration (mU/I);
- $G(t)$: interstitial glucose concentration (mg/dl);

For following the ReplayBG practices, the state equation is set as: $x(t) = \begin{bmatrix} G_p(t) & X(t) & Q_{sto1}(t) & Q_{sto2}(t) & Q_{gut}(t) & I_{sc1}(t) & I_{sc2}(t)& I_p(t) & G(t) \end{bmatrix}^T$.

### System Equations
The Non-Linear Differential Equations which describe the behavior of the system are the following:

$\dot{G_p}(t) = -[SG + \rho(G)X(t)] \cdot G_p(t) + SG \cdot G_{b} + R_{\alpha}(t)/V_G$

$\dot{X}(t) = -p_2 \cdot [X(t) - SI \cdot (I_p(t) - I_pb)]$

$\dot{Q}_{sto1}(t) = -k_{empt} \cdot Q_{sto1}(t) + m(t)$

$\dot{Q}_{sto2}(t) = k_{empt} \cdot Q_{sto1}(t) - k_{empt} \cdot Q_{sto2}(t)$

$\dot{Q}_{gut}(t) = k_{empt} \cdot Q_{sto2}(t) - k_{abs} \cdot Q_{gut}(t)$

$\dot{I}_{sc1}(t) = -k_{d}\cdot I_{sc1}(t) + i(t-\beta / V_{I})$

$\dot{I}_{sc2}(t) = k_{d}\cdot I_{sc1}(t) - k_{\alpha2} \cdot I_{sc2}(t)$

$\dot{I}_{p}(t) = k_{\alpha2} \cdot I_{sc2}(t) - k_{e} \cdot I_{p}(t)$

$\dot{G}(t) = -\frac{1}{\alpha}(G(t) - G_p(t))$

where, $k_{d}, k_{\alpha2}, k_e, \beta, k_{empt}, k_{abs}, \alpha, p_2$ are parameters describing the rate of diffusion and absorption of insulin, $V_{I}$ is the volume of insulin distribution, $R_{\alpha}(t)$ is the rate of glucose appearance in plasma, $SG$ is the glucose effectiveness, $G_b$ is the basal glucose concentration in the plasma, $V_G$ is the volume of glucose distribution, $I_{pb}$ is the basal insulin concentration in the plasma, $SI$ is the insulin sensitivity, $\rho(G)$ is a function to better represent glucose dynamics in the hypoglycemic range. 

The $I$ variables and the associated equaitons describe the absorption dynamics of exogenous insulin infusion to the plasma, the $Q$ variables describe the glucose absorption due to meals, and the last 3 equations describe the glucose-insulin kinetics subsystem.  

### Linearized Equations

We linearized the nonlinear dynamics above around the steady state $x_{ss} = \begin{bmatrix}120.0 & 0.0016 & 0.0 & 0.0 & 0.0 & 1.5081 &10.2735 & 0.2827 & 120.0\end{bmatrix}$ using a discretization step of 1 minute.
The linearized dynamics model has the form


$$\textbf{x}_{k+1} = A \textbf{x}_{k} + B\textbf{u}_{k} + h.$$
 
- $A \in \mathbb{R}^{9\times9}$ is the dynamics matrix,
- $B \in \mathbb{R}^{9\times2}$ is the input matrix, and
- $h \in \mathbb{R}^{9\times1}$ is the offset constant produced by the linearization.

<font color='red'>NOTE</font>: There are three different time intervals: the time-discretization of the simulation (1 minute), the meal time interval (5 minutes), and the controller actuation time interval (up to you).


## ReplayBG Specifics

simulate() runs a sytem simulation. It takes the following inputs:
- basal_handler: the function/controller that returns the  basal insulin injection value
- basal_handler_params: dict() with parameters defined by the control designer that can be accessed during simulation by the handler
- data_given: a pandas dataframe that describes the meal intake. The example below shows how to create the dataframe by reading the meal csv file (the control designer may change the csv file or define multiple csv files/dataframes as needed)
- meal_input_modulation_factor: constant that modules the impact of a meal (the default value should be used)

T1DModel: The T1DModel class is a part of ReplayBG's simulator and describes the glucose dynamics. It takes the following inputs:
- data: pandas dataframe which contains the preliminary meal data ("data" is not very relevant as it is overwritten with data_given when simulate() is called)
- bw: the body weight of the patient (the default value should be used)
- yts: The continuous glucose measurement (cgm) sample time (the default value should be used)
- glucose_model: the model equation to be used as measured glucose ('IG' should be used)
- is_single_meal: a flag indicating if the model will be used as single meal or multi meal (the default value (False) should be used)
- exercise: a boolean indicating if the model includes exercise (the default value (False) should be used)

The initial control design should not need most of the inputs, as InsulinCo is primarily interested in performance on the basic/default case. For the bonus, you could consider changing some of the inputs. Keep in mind, InsulinCo is primarily interested in new control methods rather than exhaustive studies, so any parameter changes should be used/changed to demonstrate the value of a given controller, not for the sake of an exhaustive study.

Below are some additional functions from InsulinCo intended to make the ReplayBG simulator easier to use.

**get_linearization():** InsulinCO has provided this function to you to help design your controllers. It linearizes the dynamics around a given operating point but does *not* take into account the basal insulin injection delay. This means that for a given time index $t$ and a delay $tau$, the basal insulin injection value returned by the controller will only be seen by the system at the time step $t+tau$. This also means that for the same $t$, the control actions at the time steps between $t$ and $t+tau-1$ are already defined and cannot be changed. 

<font color='red'>NOTE</font>: The ReplayBG simulator has a delay that is not included in the linearization. This delay must be factored in separately from the linearizaed dynamics described by the A matrix.

**get_discretized_meals():** InsulinCo has provided this function to you to help interface between the ReplayBG simulator and the csv (that you may edit) that defines the meals. get_discretized_meals() is helpful because of how ReplayBG processes the meal data. Specifically, ReplayBG does some operations on the values written in the csv file to get the value of $m(t)$. Thus, to access this internal variable of ReplayBG, InsulinCo created this function which returns a vector with the all the $m[k]$ in a certain interval of time and according to a discretization time step. 
