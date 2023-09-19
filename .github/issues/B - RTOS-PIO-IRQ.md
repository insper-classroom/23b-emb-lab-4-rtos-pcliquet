> Issue:
>
> - Você só deve fechar as issues que foram 100% concluidas.
> - Se não concluiu 100% a issues, você pode fazer um comentário que ireimos analisar!

| Projeto |
| ------| 
| `RTOS-PIO-IRQ`|       

No código dos botões, no lugar de usar um semáforo para cada botão, vamos usar uma única fila que irá realizar a comunicação entre os callbacks dos botões e a task `task_but`. 

Tarefa:

1. Crie mais uma uma fila
1. Cada callback deve colocar o valor referente ao ID do botão que foi apertado
1. A `task_but` recebe o valor e repassa para a `task_led`

Qualidade de Código:

- Lembre de verificar se os testes de qualidade de código estão passando.

Esperado:

![](https://raw.githubusercontent.com/Insper/ComputacaoEmbarcada/master/docs-src/navigation/Labs/Lab_RTOS/imgs/pio-btns-B.svg)

Extra:

==Você saberia informar na fila se o botão foi apertado ou liberado?==
