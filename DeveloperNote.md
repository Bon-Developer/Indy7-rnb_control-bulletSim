## DeveloperNote

### Nominal Jacobian in panda
* **Panda** SDK에서 nominal Q에 대한 M, C 계산에 다른 여러 파라미터가 필요해 현재는 real Q에 의한 M, C만을 계산한다.

* **Panda** SDK에서 함수 정의를 타고 들어가 보면 Q를 파라미터로 넘겨받는 Jacobian function이 넘겨받은 Q를 쓰지 않고 로봇의 현재 Q를 사용해 Jacobian을 계산하는 것을 확인할 수 있다. 현재는 nominal Jacobian은 계산하지 않고 현재 Q로 얻은 Jacobian을 nominal Jacobian에 똑같이 넣고 있다. 이 부분을 고치기 위해서는 안쪽에서 사용되는 로봇 모델과 Jacobian 함수를 외부에서 꺼내 직접 계산하거나 해당 부분 헤더 파일을 변경해 수정이 되는지 확인해봐야 한다.

* **M, C**를 현재는 nominal 값 만을 계산해 넣고 있다. 이 때문에 모델 기반이 아닌 PD 제어기의 경우에 제대로 된 M,C 값을 사용하지 못하고 있다. **M, C**의 nomial, real를 구분해 계산해 넘겨줄 필요가 있다.