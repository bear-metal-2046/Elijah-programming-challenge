# 2024 Programming Challenge

> This challenge focuses on reimplementing the 2024 Collector and Indexer, testing both technical skills and teamwork.
> New members will work with boilerplate subsystems and a pre-built chassis to build their solution, emphasizing
> learning,
> adaptability, and collaboration. Returning members will design state machines for teleop and autonomous modes, working
> solo initially before mentoring new members. Performance will be evaluated based on implementation, teamwork, and
> attitude, ensuring a well-rounded assessment of skills and collaboration.
>
> New members will be provided with a boilerplate template to start them off, while returning members will need to start
> from scratch. All members must implement the functionality described in the boilerplate repository’s README.

> [!CAUTION]
> Returning members are not allowed to use the boilerplate in this repository. All members are **not allowed to use
2024's codebase**[^1].

[^1]: This is subject to change for new members, depending on how the challenge goes.

> [!IMPORTANT]
> The collector **must** be zeroed, prior to setting any positions.

> [!IMPORTANT]
> Before deploying, you **must have another member of the same level look over your code** and new members must have a lead as well.

## GitHub Usage

All members should make frequent and detailed commits. The more information we have to assess you on the better, if the
end product is non-functional but your commit history and code shows understanding and problem-solving, this will be
taken into account during cuts.

### New Members

This is a template repository for you to use. Go to the top right of the website and click
`Use this template > Create a new repository`.

> [!NOTE]
> If you feel that the boilerplate does not fit with how you'd like to architecture your code, feel free to remove it.

## Timeline

All members are required to complete an entry in [TIMELINE.md](TIMELINE.md) every day, detailing what they did.

| Day 1 (11/18) | Day 2 (11/19) | Day 3 (11/20) | Day 4 (11/21) | Day 5 (11/22) | Day 6 (11/23) |
|---------------|---------------|---------------|---------------|---------------|---------------|
| Setup | Returning members can test on robot | All members can test on robot & Returning members projects due | Returning members help new members | <-- | New members projects due |

## States

### Indexer

| State         | Description                                                                                                            |
|---------------|------------------------------------------------------------------------------------------------------------------------|
| **Disabled**  | Default state; The indexer is empty and not enabled                                                                    |
| **Intaking**  | The indexer is intaking into the shooter, ran while empty and collector is collecting                                  |
| **Indexing**  | The first beam brake was tripped while **Intaking**, slows to a lesser velocity until the second beam brake is tripped |
| **Collected** | A note is current in the indexer and has been indexed                                                                  |
| **Ejecting**  | Regardless of the collection status, empty the indexer towards the collector                                           |

### Collector

#### Collection State

| State          | Description                                                                        |
|----------------|------------------------------------------------------------------------------------|
| **Disabled**   | Default state; The collector motors are not enabled                                |
| **Collecting** | The collector is **Deployed** and collector motors are collecting into the indexer |
| **Ejecting**   | Collector motors spin away from the indexer                                        |

#### Deployment State

| State        | Description                                                                    |
|--------------|--------------------------------------------------------------------------------|
| **Stowed**   | Default state; The collector is in its upwards position                        |
| **Deployed** | Collector is down to the ground (but not dragging), ready to collect notes     |
| **Eject**    | Collector is slightly up to allow for ejected notes to smoothly exit the robot |

## Controller Bindings

### Drive Controller

| Binding           | Function                |
|-------------------|-------------------------|
| **Left Bumper**   | Toggle collector deploy |
| **Left Trigger**  | Collect when held       |
| **DPad/POV Left** | Eject                   |
