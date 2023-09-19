> Issue:
>
> - Você só deve fechar as issues que foram 100% concluidas.
> - Se não concluiu 100% a issues, você pode fazer um comentário que ireimos analisar!

| Projeto         |
|-----------------|
| `RTOS-AFEC-IRQ` |

A ideia agora é criarmos uma task intermediária (`task_proc`) que irá fazer o processamento dos dados recebidos pelo ADC. 

A `task_proc` deverá calcular a média móvel (N=10) dos dados do ADC e então enviar esses dados para para a task `task_adc` via uma nova fila, com o objetivo de ser exibido. A ideia da média móvel é aplicar um filtro passa baixas para remover ruídos do sinal.

- Referência: https://www.analog.com/media/en/technical-documentation/dsp-book/dsp_book_ch15.pdf

Tarefas/dicas:

1. Modifique o código adicionando a task_proc
1. Faça a leitura do dado do AFEC nessa nova tarefa 
1. Crie uma nova fila, e envie o dado da task_proc para a task_adc (dado bruto, ainda sem processamento)
1. Na task_adc exiba o dado via printf
1. Agora na task proc calcule a média móvel e o envie para a task_adc.

Qualidade de Código:

- Lembre de verificar se os testes de qualidade de código estão passando.

Esperado:

![](https://raw.githubusercontent.com/Insper/ComputacaoEmbarcada/master/docs-src/navigation/Labs/Lab_RTOS/imgs/diagrama2.svg)
