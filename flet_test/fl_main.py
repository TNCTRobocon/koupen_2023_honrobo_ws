import flet as ft

def main(page: ft.Page):
    page.title = "Flet counter example"
    page.vertical_alignment = ft.MainAxisAlignment.CENTER

    # 値を動的にしたい部分のControlインスタンスを作成
    txt_number = ft.TextField(value="0", text_align=ft.TextAlign.RIGHT, width=100)

    # マイナスボタンクリック時の処理
    def minus_click(e):
        #Controlインスタンスのvalueプロパティに代入
        txt_number.value = str(int(txt_number.value) - 1) 
        #ページを更新。(txt_number.update()としても良い。updateは子要素に伝達する)
        page.update() 

    # プラスボタンクリック時の処理
    def plus_click(e):
        txt_number.value = str(int(txt_number.value) + 1)
        page.update()

    page.add(
        ft.Row(
            [
                ft.IconButton(ft.icons.REMOVE, on_click=minus_click),
                txt_number,
                ft.IconButton(ft.icons.ADD, on_click=plus_click),
            ],
            alignment=ft.MainAxisAlignment.CENTER,
        )
    )

ft.app(target=main)