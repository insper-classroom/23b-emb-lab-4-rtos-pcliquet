> Issue:
>
> - Você só deve fechar as issues que foram 100% concluidas.
> - Se não concluiu 100% a issues, você pode fazer um comentário que ireimos analisar!

| Projeto |
| ------| 
| `RTOS-PIO-IRQ`|   

Agora vamos usar os outros dois botões da placa OLED, cada botão vai decrementar a frequência por uma constante diferente:

- BTN1: 10
- BTN2: 5
- BTN3: 1

Para isso você deverá:

1. configurar o callback para cada botão novo
1. Criar um novo semáforo para cada botão
1. Ler os semáforos na `task_but` e processar os valores

Tarefa:

1. Agora faca o mesmo para os outros dois botões da placa OLED que faltaram, cada um modificando a frequência com um valor diferente.


Qualidade de Código:

- Lembre de verificar se os testes de qualidade de código estão passando.

Esperado:

![](https://raw.githubusercontent.com/Insper/ComputacaoEmbarcada/master/docs-src/navigation/Labs/Lab_RTOS/imgs/pio-btns.svg)
