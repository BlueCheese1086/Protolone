| **State**        | **Description**                                                                                             | **Transitions**                                                                                                                                                             |
|------------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Idle**         | Do nothing                                                                                               | Intake request → **Intake**  <br> Note detected → **Ready**  <br> Manual request → **Manual**                                                                                 |
| **Intake**       | Intake a note                                                                                | Idle request → **Idle**  <br> Note detected → **Ready**  <br> Manual request → **Manual**                                                                                      |
| **Eject**        | Eject the note                                                                                     | Note no longer detected → **Idle**  <br> Manual request → **Manual**                                                                                                         |
| **Ready**        | Note inside robot                                                                                           | Note no longer detected → **Idle**  <br> Idle request → **Eject**  <br> Auto score request → **Auto score**  <br> Manual score request → **Manual score**  <br> Manual request → **Manual**|
| **Auto score**   | Lock heading to target <br> Run flywheel if within range <br> May auto-align         | Note no longer detected → **Idle**  <br> Idle request → **Eject**  <br> Manual score request → **Manual score**  <br> Score request and correct parameters → **Score**  <br> Manual request → **Manual**   |
| **Manual score** | Manual control over shooter                                                                                 | Note no longer detected → **Idle**  <br> Idle request → **Eject**  <br> Auto score request → **Auto score**  <br> Score request → **Score**  <br> Manual request → **Manual**         |
| **Score**        | Shoot the note                                                                                       | Note no longer detected → **Idle**  <br> Manual request → **Manual**                                                                                                         |
| **Manual**       | Manual control over robot                                                                                   | Exit manual request → **Ready**                                                                                                                                                       |

```mermaid
stateDiagram-v2
    [*] --> Idle

    Idle --> Intake: Intake request
    Idle --> Ready: Note detected
    Idle --> Manual: Manual request

    Intake --> Idle: Idle request
    Intake --> Ready: Note detected
    Intake --> Manual: Manual request

    Eject --> Idle: Note no longer detected
    Eject --> Manual: Manual request

    Ready --> Idle: Note no longer detected
    Ready --> Eject: Idle request
    Ready --> AutoScore: Auto score request
    Ready --> ManualScore: Manual score request
    Ready --> Manual: Manual request

    AutoScore --> Idle: Note no longer detected
    AutoScore --> Eject: Idle request
    AutoScore --> ManualScore: Manual score request
    AutoScore --> Score: Score request + correct parameters
    AutoScore --> Manual: Manual request

    ManualScore --> Idle: Note no longer detected
    ManualScore --> Eject: Idle request
    ManualScore --> AutoScore: Auto score request
    ManualScore --> Score: Score request
    ManualScore --> Manual: Manual request

    Score --> Idle: Note no longer detected
    Score --> Manual: Manual request

    Manual --> Idle: Idle request
```
