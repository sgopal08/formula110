## Laboratory Notebook 1

**Date and time:** 8/30 11:30 AM

**Participants and contributions:** Yewon and Sanjana

## Question or objective

What training methods would be the most effective for training a car controller that is both fast and reliable?

## What we investigated or changed

We investigated the different training approaches, including Reactive Control and Parameter Optimization, Imitation Learning, Evolutionary Computation and Neuroevolution, Model-Free Reinforcement Learning, Planning and Model-Predictive Control, Hybrid Approaches, and Learned Dynamics or Model-Based Learning. We interacted with AI to determine what may be the best option(s) and why the others fall short of our objective.

## Evidence

[ChatGPT discussion](https://chatgpt.com/share/6a944b96-5b4c-83ea-90f5-4d7e1d97560b)

## What we observed

We observed that reactive control would be a good place to start, since we can explicitly define the sensors and how it affects the steering and throttle. It can also allow us to understand why some things occurred and diagnose issues earlier on, such as braking too much or turning too sharply. However, since manually determining the most optimal values isn’t very scalable, we looked into how CMA-ES could help by automatically evaluating the different parameter combinations based on performance.

## Decision and rationale

We decided to try experimenting with reactive control and CMA-ES, with the possibility of combining them into one refined controller. Initially, we would construct a reactive controller that manages the basic driving behaviors like steering according to track geometry and sensor readings, slowing down when there’s a curve, accelerating when possible, and maintaining an optimal position on the track. After establishing a baseline performance, we would use CMA-ES to refine the parameters, such as sensor weights, steering gains, throttle levels, corner-speed thresholds, braking distances, and steering smoothing. We agreed that this approach would be the most efficient since it has a good balance between implementation difficulty, computational efficiency, and potential racing performance.

## Next steps

1. Familiarize ourselves with the codebase, racing environment, sensors, and controller interface.
2. Determine which sensor readings are most useful for identifying the center of the track, upcoming curves, and safe acceleration opportunities.
3. Implement a stage one reactive controller that can reliably drive around the track.
4. Define quantitative evaluation metrics, including progress/distance traveled, track-completion rate, lap time or average speed, damage/crashes, and consistency across starting-point seeds.
5. Select a fixed collection of seeds so different controllers can initially be compared under equivalent conditions.
6. Record the baseline performance of the manually tuned reactive controller.
7. Learn how CMA-ES works and determine which controller parameters should be optimized.
8. Develop a fitness function that rewards both speed/progress and reliability rather than allowing the optimizer to favor controllers that are extremely fast but frequently crash.
9. Compare the experimental approaches using the same evaluation conditions and use the results to decide what should be carried forward into the refinement stage.
"""

