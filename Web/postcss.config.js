/** @type {import('postcss-load-config').Config} */
import presetEnv from 'postcss-preset-env'
import cssnano from 'cssnano'

const config = {
    plugins: [
        presetEnv({
            autoprefixer: true,
            stage: 3,
            features: {
                'nesting-rules': true,
            }
        }),
        cssnano({
            preset: 'advanced',
        })
    ]
}

export default config